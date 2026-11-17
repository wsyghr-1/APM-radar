#!/usr/bin/env python3
"""Build radar-enabled firmware batches and collect a complete release."""

import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.parse import quote

from radar_release_boards import ROOT, validate_board, validate_version


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def run(command, log, **kwargs):
    print("+ " + " ".join(map(str, command)), flush=True)
    log.write("\n+ " + " ".join(map(str, command)) + "\n")
    log.flush()
    return subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                          check=True, **kwargs)


def read_config_cache(board):
    """Read literal Waf compiler settings without executing the cache file."""
    path = ROOT / "build/c4che" / f"{board}_cache.py"
    wanted = {"CXX", "DEFINES", "CXXFLAGS", "CPU_FLAGS"}
    result = {}
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id in wanted:
                result[target.id] = ast.literal_eval(node.value)
    if not wanted.issubset(result):
        raise ValueError(f"Missing compiler settings in {path}")
    return result


def radar_skip_reason(board, log):
    """Return why a board cannot provide real-hardware EKF3 radar navigation.

    C preprocessing avoids pulling in C++ HAL implementation headers. Compiler
    defines and CPU flags are taken from this board's configured Waf cache.
    The linked ELF is checked independently after the actual C++ build.
    """
    cache = read_config_cache(board)
    flags = [flag for flag in cache["CXXFLAGS"] if flag.startswith(("-D", "-U"))]
    source = (
        "#include <AP_Vehicle/AP_Vehicle_Type.h>\n"
        "#include <AP_RadarOdom/AP_RadarOdom_config.h>\n"
        "#include <AP_NavEKF3/AP_NavEKF3_feature.h>\n"
    )
    requirements = (
        ("HAL_RADARODOM_ENABLED", True),
        ("HAL_NAVEKF3_AVAILABLE", True),
        ("EK3_FEATURE_EXTERNAL_NAV", True),
        ("AP_SIM_ENABLED", False),
    )
    for macro, _ in requirements:
        # Feature macros may expand to unparenthesized compound expressions.
        source += (f"#if ({macro})\nRADAR_RELEASE_{macro}_1\n"
                   f"#else\nRADAR_RELEASE_{macro}_0\n#endif\n")
    command = cache["CXX"] + cache["CPU_FLAGS"] + flags + [
        "-E", "-P", "-x", "c", "-DAPM_BUILD_DIRECTORY=2",
        "-I", str(ROOT / "libraries"), "-I", str(ROOT / "build" / board),
        "-include", str(ROOT / "build" / board / "ap_config.h"),
    ] + ["-D" + definition for definition in cache["DEFINES"]] + ["-"]
    completed = subprocess.run(command, cwd=ROOT, input=source, text=True,
                               stdout=subprocess.PIPE, stderr=log, check=True)
    markers = set(completed.stdout.splitlines())
    reasons = []
    for macro, wanted in requirements:
        enabled = f"RADAR_RELEASE_{macro}_1" in markers
        disabled = f"RADAR_RELEASE_{macro}_0" in markers
        if enabled == disabled:
            raise ValueError(f"Missing or ambiguous release feature marker for {macro}")
        log.write(f"{macro}: {int(enabled)}\n")
        if enabled != wanted:
            reasons.append(f"{macro}={int(enabled)}")
    log.flush()
    return ", ".join(reasons) if reasons else None


def check_radar_elf(elf, log):
    symbols = subprocess.run(["arm-none-eabi-nm", "-C", "--defined-only", str(elf)],
                             cwd=ROOT, text=True, stdout=subprocess.PIPE,
                             stderr=log, check=True).stdout
    required = ("AP_RadarOdom::var_info", "AP_RadarOdom::handle_odometry(")
    missing = [name for name in required if name not in symbols]
    if missing:
        raise ValueError("Radar symbols missing from linked firmware: " + ", ".join(missing))


def copy_nonempty(source, destination):
    if not source.is_file() or source.stat().st_size == 0:
        raise ValueError(f"Missing or empty build product: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def build_batch(args):
    boards = json.loads(args.boards_json)
    if not isinstance(boards, list) or not boards or len(set(boards)) != len(boards):
        raise ValueError("Expected a nonempty list of unique boards")
    for board in boards:
        validate_board(board)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("Batch output directory must be empty")
    (output / "logs").mkdir(exist_ok=True)
    result = {"version": validate_version(args.version), "commit": args.commit,
              "boards": []}
    result_path = output / "results.json"
    write_json(result_path, result)
    for board in boards:
        record = {"board": board, "status": "failed"}
        log_path = output / "logs" / f"{board}.log"
        print(f"::group::{board}", flush=True)
        with log_path.open("w", encoding="utf-8") as log:
            try:
                run(["./waf", "configure", "--board", board, "--no-submodule-update"], log)
                reason = radar_skip_reason(board, log)
                if reason is not None:
                    record.update(status="skipped", reason=reason)
                else:
                    run(["./waf", "copter", "-j", str(args.jobs)], log)
                    build = ROOT / "build" / board
                    check_radar_elf(build / "bin/arducopter", log)
                    firmware_name = f"{board}_ArduCopter.apj"
                    copy_nonempty(build / "bin/arducopter.apj", output / firmware_name)
                    record["firmware"] = firmware_name
                    hwdef_bl = ROOT / "libraries/AP_HAL_ChibiOS/hwdef" / board / "hwdef-bl.dat"
                    if hwdef_bl.is_file():
                        # Build independently; do not overwrite repository bootloader
                        # images or publish a combined image with a different loader.
                        run(["./waf", "configure", "--board", board, "--bootloader",
                             "--no-submodule-update"], log)
                        run(["./waf", "clean"], log)
                        run(["./waf", "bootloader", "-j", str(args.jobs)], log)
                        name = f"{board}_bootloader.bin"
                        copy_nonempty(build / "bin/AP_Bootloader.bin", output / name)
                        record["bootloader"] = name
                    else:
                        record["bootloader_note"] = "No board-specific hwdef-bl.dat; no bootloader published"
                    record["status"] = "built"
                if args.clean:
                    run(["./waf", "clean"], log)
            except (subprocess.CalledProcessError, OSError, ValueError) as error:
                record.update(status="failed", reason=str(error))
                log.write(f"\nFailure: {error}\n")
        result["boards"].append(record)
        write_json(result_path, result)
        print(f"{board}: {record['status']}", flush=True)
        if record["status"] == "failed":
            print(f"Failure: {record['reason']}", flush=True)
            print("\n".join(log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-60:]))
        print("::endgroup::", flush=True)
    return int(any(record["status"] == "failed" for record in result["boards"]))


def collect(args):
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    validate_version(plan["version"])
    expected = set(plan["candidates"])
    records = {}
    sources = {}
    for path in sorted(args.results.glob("radar-batch-*/results.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        if (data["version"], data["commit"]) != (plan["version"], plan["commit"]):
            raise ValueError(f"Version or commit mismatch: {path}")
        for record in data["boards"]:
            board = record["board"]
            if board not in expected or board in records:
                raise ValueError(f"Unexpected or duplicate result: {board}")
            if record["status"] not in ("built", "skipped"):
                raise ValueError(f"Failed board: {board}")
            records[board] = record
            sources[board] = path.parent
    if set(records) != expected:
        raise ValueError("Missing board results: " + ", ".join(sorted(expected - set(records))))
    built = [records[name] for name in sorted(records) if records[name]["status"] == "built"]
    if not built:
        raise ValueError("No radar-enabled firmware was built")
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        raise ValueError("Release output directory must be empty")
    for record in built:
        board = record["board"]
        for key, wanted in (("firmware", f"{board}_ArduCopter.apj"),
                            ("bootloader", f"{board}_bootloader.bin")):
            if key not in record:
                if key == "firmware":
                    raise ValueError(f"No firmware recorded for {board}")
                continue
            if record[key] != wanted:
                raise ValueError(f"Unexpected asset name for {board}")
            copy_nonempty(sources[board] / wanted, args.output / wanted)
    (args.output / "supported-boards.txt").write_text(
        "\n".join(record["board"] for record in built) + "\n", encoding="utf-8")
    skipped = [f"{name}: {record['reason']}" for name, record in sorted(records.items())
               if record["status"] == "skipped"]
    skipped.extend(f"{item['board']}: {item['reason']}" for item in plan["excluded"])
    (args.output / "skipped-boards.txt").write_text("\n".join(skipped) + "\n", encoding="utf-8")
    manifest = dict(plan)
    manifest["results"] = [records[name] for name in sorted(records)]
    write_json(args.output / "build-manifest.json", manifest)
    base_url = f"https://github.com/{args.repository}"
    version = quote(plan["version"], safe="")
    if args.previous_tag:
        previous = quote(args.previous_tag, safe="")
        changelog_url = f"{base_url}/compare/{previous}...{version}"
    else:
        changelog_url = f"{base_url}/commits/{version}"
    notes = f"**Full Changelog**: {changelog_url}\n"
    (args.output / "RELEASE_NOTES.md").write_text(notes, encoding="utf-8")
    sums = []
    for path in sorted(args.output.iterdir()):
        sums.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}")
    (args.output / "SHA256SUMS").write_text("\n".join(sums) + "\n", encoding="utf-8")
    print(f"Collected {len(built)} radar-enabled boards for {plan['version']}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build")
    build.add_argument("--boards-json", required=True)
    build.add_argument("--version", required=True)
    build.add_argument("--commit", required=True)
    build.add_argument("--output", type=Path, required=True)
    build.add_argument("--jobs", type=int, default=min(os.cpu_count() or 2, 4))
    build.add_argument("--clean", action="store_true", help="Run waf clean after each successful board")
    build.set_defaults(handler=build_batch)
    gather = subparsers.add_parser("collect")
    gather.add_argument("--plan", type=Path, required=True)
    gather.add_argument("--results", type=Path, required=True)
    gather.add_argument("--output", type=Path, required=True)
    gather.add_argument("--repository", required=True, help="GitHub repository as owner/name")
    gather.add_argument("--previous-tag", help="Previous published release tag; omit for the first release")
    gather.set_defaults(handler=collect)
    args = parser.parse_args()
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
