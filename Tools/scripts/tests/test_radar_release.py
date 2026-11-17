"""Offline tests for release planning, feature checks and completeness gates."""

import argparse
import ast
from contextlib import redirect_stdout
import fnmatch
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest.mock import Mock, patch
import warnings

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import radar_release_boards as boards  # noqa: E402
import radar_release_build as build  # noqa: E402


class ReleaseTests(unittest.TestCase):
    def test_release_upload_only_firmware(self):
        workflow = (boards.ROOT / ".github/workflows/release_firmware.yml").read_text()
        step = workflow.split("      - name: Publish versioned release\n", 1)[1]
        body = step.split("        run: |\n", 1)[1]
        lines = []
        for line in body.splitlines():
            if line.strip() and not line.startswith("          "):
                break
            lines.append(line)
        script = textwrap.dedent("\n".join(lines))
        # Exercise the real workflow shell without contacting GitHub.
        fake_gh = (
            "gh() {\n"
            "  case \"$2\" in\n"
            "    view) if [ \"$TEST_DRAFT_EXISTS\" = 1 ]; then echo true; else return 1; fi ;;\n"
            "    create) printf '%s\\n' \"$@\" > created-arguments.txt ;;\n"
            "    upload) printf '%s\\n' \"$@\" > uploaded-arguments.txt ;;\n"
            "    edit) printf '%s\\n' \"$@\" > edited-arguments.txt ;;\n"
            "    *) return 1 ;;\n"
            "  esac\n"
            "}\n"
        )
        for with_app, with_bootloader, draft_exists in (
                (True, True, True), (True, False, True), (True, True, False), (False, False, True)):
            with self.subTest(app=with_app, bootloader=with_bootloader, draft=draft_exists), \
                    tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                assets = root / "release-assets"
                assets.mkdir()
                names = ["supported-boards.txt", "skipped-boards.txt", "build-manifest.json",
                         "SHA256SUMS", "RELEASE_NOTES.md", "unexpected.bin", "unexpected.apj"]
                expected = []
                if with_app:
                    expected.append("release-assets/Pixhawk6X_ArduCopter.apj")
                if with_bootloader:
                    expected.append("release-assets/Pixhawk6X_bootloader.bin")
                names += [Path(name).name for name in expected]
                for name in names:
                    (assets / name).write_bytes(b"test")
                completed = subprocess.run(["bash", "-c", fake_gh + script], cwd=root,
                                           env=dict(os.environ, RELEASE_VERSION="v4.7.0-dev-1.0.0",
                                                    TEST_DRAFT_EXISTS=str(int(draft_exists))),
                                           capture_output=True, text=True)
                captured = root / "uploaded-arguments.txt"
                if with_app:
                    self.assertEqual(completed.returncode, 0, completed.stderr)
                    self.assertEqual(captured.read_text().splitlines(),
                                     ["release", "upload", "v4.7.0-dev-1.0.0", *expected, "--clobber"])
                    edited = (root / "edited-arguments.txt").read_text().splitlines()
                    self.assertIn("--draft=false", edited)
                    self.assertIn("--prerelease=false", edited)
                    if not draft_exists:
                        created = (root / "created-arguments.txt").read_text().splitlines()
                        self.assertIn("--draft", created)
                        self.assertIn("--prerelease=false", created)
                else:
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertFalse(captured.exists())

    def test_periph_radar_define_does_not_leak_into_copter(self):
        path = boards.ROOT / "Tools/AP_Periph/wscript"
        function = next(node for node in ast.parse(path.read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == "build")
        namespace = {"fnmatch": fnmatch}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
        # ARKV6X is included by the coarse AP_PERIPH string search, even though
        # its normal Copter configuration is not an AP_Periph build.
        for board, periph, heavy, listed in (
                ("ARKV6X-bdshot", False, False, True),
                ("Pixhawk6X", False, False, False),
                ("CubeNode-ETH", True, False, True),
                ("periph-heavy", True, True, True)):
            with self.subTest(board=board):
                context = Mock()
                context.env = argparse.Namespace(BOARD=board, AP_PERIPH=periph,
                                                 AP_PERIPH_HEAVY=heavy, DEFINES=[])
                context.ap_periph_boards.return_value = [board] if listed else []
                context.ap_get_all_libraries.return_value = []
                namespace["build"](context)
                self.assertEqual("HAL_RADARODOM_ENABLED=0" in context.env.DEFINES, periph)

    def test_version(self):
        self.assertEqual(boards.validate_version("v4.7.0-dev-1.0.0"), "v4.7.0-dev-1.0.0")
        for value in ("v1.0.0", "v4.7.0-dev-1.0", "v4.7.0-dev-01.0.0", "../../file"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                boards.validate_version(value)

    def test_inventory(self):
        # The upstream board-list reader leaves input files unclosed.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            plan = boards.make_plan("v4.7.0-dev-1.0.0", "test-commit", 20)
        self.assertIn("Pixhawk6X", plan["candidates"])
        assigned = [name for batch in plan["matrix"]["include"] for name in batch["boards"]]
        self.assertCountEqual(assigned, plan["candidates"])
        self.assertEqual(len(assigned), len(set(assigned)))
        self.assertEqual(len(plan["matrix"]["include"]), 20)
        self.assertNotIn("SITL_x86_64_linux_gnu", assigned)
        self.assertNotIn("Pixhawk6X-bdshot", assigned)
        self.assertFalse(any(name.lower().endswith("-bdshot") for name in assigned))
        excluded = {item["board"]: item["reason"] for item in plan["excluded"]}
        self.assertEqual(excluded["Pixhawk6X-bdshot"], "bdshot variants are not published")
        self.assertEqual(excluded["ARKV6X-bdshot"], "bdshot variants are not published")
        with self.assertRaises(ValueError):
            boards.validate_board("../Pixhawk6X")

    def test_radar_preprocessor(self):
        cache = {"CXX": ["compiler"], "CPU_FLAGS": [], "CXXFLAGS": [], "DEFINES": []}
        features = {"HAL_RADARODOM_ENABLED": 1, "HAL_NAVEKF3_AVAILABLE": 1,
                    "EK3_FEATURE_EXTERNAL_NAV": 1, "AP_SIM_ENABLED": 0}
        for changed in (None, *features):
            values = dict(features)
            if changed:
                values[changed] = 1 - values[changed]
            expected = f"{changed}={values[changed]}" if changed else None
            markers = "\n".join(f"RADAR_RELEASE_{name}_{value}" for name, value in values.items())
            with patch.object(build, "read_config_cache", return_value=cache), \
                    patch.object(build.subprocess, "run", return_value=argparse.Namespace(stdout=markers)):
                with tempfile.TemporaryFile(mode="w+") as log:
                    self.assertEqual(build.radar_skip_reason("Pixhawk6X", log), expected)

    def test_radar_preprocessor_rejects_missing_markers(self):
        cache = {"CXX": ["compiler"], "CPU_FLAGS": [], "CXXFLAGS": [], "DEFINES": []}
        for markers in ("", "RADAR_RELEASE_HAL_RADARODOM_ENABLED_0\n"
                        "RADAR_RELEASE_HAL_RADARODOM_ENABLED_1\n"):
            with patch.object(build, "read_config_cache", return_value=cache), \
                    patch.object(build.subprocess, "run", return_value=argparse.Namespace(stdout=markers)), \
                    tempfile.TemporaryFile(mode="w+") as log:
                with self.assertRaises(ValueError):
                    build.radar_skip_reason("Pixhawk6X", log)

    def test_radar_features_with_real_preprocessor(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            self.skipTest("Host GCC is not available")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            headers = {
                "AP_Vehicle/AP_Vehicle_Type.h": "",
                "AP_RadarOdom/AP_RadarOdom_config.h": "#define HAL_RADARODOM_ENABLED (1 && 1)\n",
                "AP_NavEKF3/AP_NavEKF3_feature.h": (
                    "#define HAL_NAVEKF3_AVAILABLE 1 && 1\n"
                    "#ifndef EK3_FEATURE_EXTERNAL_NAV\n#define EK3_FEATURE_EXTERNAL_NAV 0 || 1\n#endif\n"
                    "#ifndef AP_SIM_ENABLED\n#define AP_SIM_ENABLED 0\n#endif\n"),
            }
            for name, content in headers.items():
                path = root / "libraries" / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            config = root / "build/test/ap_config.h"
            config.parent.mkdir(parents=True)
            config.write_text("")
            cache = {"CXX": [compiler], "CPU_FLAGS": [], "CXXFLAGS": [], "DEFINES": []}
            cases = (("", None),
                     ("#define EK3_FEATURE_EXTERNAL_NAV 0\n", "EK3_FEATURE_EXTERNAL_NAV=0"),
                     ("#define AP_SIM_ENABLED 1\n", "AP_SIM_ENABLED=1"))
            for extra, expected in cases:
                config.write_text(extra)
                with patch.object(build, "ROOT", root), \
                        patch.object(build, "read_config_cache", return_value=cache), \
                        tempfile.TemporaryFile(mode="w+") as log:
                    self.assertEqual(build.radar_skip_reason("test", log), expected)

    def test_elf_requires_radar(self):
        for symbols, valid in (("AP_RadarOdom::var_info\nAP_RadarOdom::handle_odometry(args)", True),
                               ("AP_RadarOdom::var_info", False), ("", False)):
            with patch.object(build.subprocess, "run", return_value=argparse.Namespace(stdout=symbols)):
                if valid:
                    build.check_radar_elf(Path("test.elf"), None)
                else:
                    with self.assertRaises(ValueError):
                        build.check_radar_elf(Path("test.elf"), None)

    def test_disabled_board_is_skipped_before_build(self):
        with tempfile.TemporaryDirectory() as temp:
            args = argparse.Namespace(boards_json='["Pixhawk6X"]', output=Path(temp) / "out",
                                      version="v4.7.0-dev-1.0.0", commit="test", jobs=2, clean=False)
            with patch.object(build, "run") as runner, \
                    patch.object(build, "radar_skip_reason", return_value="EK3_FEATURE_EXTERNAL_NAV=0"):
                self.assertEqual(build.build_batch(args), 0)
                self.assertEqual(runner.call_count, 1)
            result = json.loads((args.output / "results.json").read_text())
            self.assertEqual(result["boards"][0]["status"], "skipped")
            self.assertEqual(result["boards"][0]["reason"], "EK3_FEATURE_EXTERNAL_NAV=0")

    def test_build_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as temp:
            args = argparse.Namespace(boards_json='["Pixhawk6X"]', output=Path(temp) / "out",
                                      version="v4.7.0-dev-1.0.0", commit="test", jobs=2, clean=False)
            output = io.StringIO()
            with redirect_stdout(output), \
                    patch.object(build, "run", side_effect=subprocess.CalledProcessError(1, "waf")):
                self.assertEqual(build.build_batch(args), 1)
            result = json.loads((args.output / "results.json").read_text())
            self.assertEqual(result["boards"][0]["status"], "failed")
            self.assertIn(result["boards"][0]["reason"], output.getvalue())
            log = args.output / "logs/Pixhawk6X.log"
            self.assertIn(result["boards"][0]["reason"], log.read_text())

    def test_enabled_board_with_and_without_bootloader(self):
        for has_bootloader in (False, True):
            with self.subTest(bootloader=has_bootloader), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                products = root / "build/Pixhawk6X/bin"
                products.mkdir(parents=True)
                (products / "arducopter.apj").write_bytes(b"application")
                if has_bootloader:
                    hwdef = root / "libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X"
                    hwdef.mkdir(parents=True)
                    (hwdef / "hwdef-bl.dat").write_text("# test\n")
                args = argparse.Namespace(boards_json='["Pixhawk6X"]', output=root / "out",
                                          version="v4.7.0-dev-1.0.0", commit="test", jobs=2, clean=False)
                with patch.object(build, "ROOT", root), patch.object(build, "run") as runner, \
                        patch.object(build, "radar_skip_reason", return_value=None), \
                        patch.object(build, "check_radar_elf"):
                    def mock_run(command, log):
                        if command[1] == "bootloader":
                            (products / "AP_Bootloader.bin").write_bytes(b"bootloader")
                    runner.side_effect = mock_run
                    self.assertEqual(build.build_batch(args), 0)
                    self.assertEqual(runner.call_count, 5 if has_bootloader else 2)
                result = json.loads((args.output / "results.json").read_text())["boards"][0]
                self.assertEqual(result["status"], "built")
                self.assertEqual(result["firmware"], "Pixhawk6X_ArduCopter.apj")
                self.assertEqual((args.output / result["firmware"]).read_bytes(), b"application")
                self.assertEqual("bootloader" in result, has_bootloader)
                if has_bootloader:
                    self.assertEqual(result["bootloader"], "Pixhawk6X_bootloader.bin")
                    self.assertEqual((args.output / result["bootloader"]).read_bytes(), b"bootloader")

    def test_bootloader_failure_blocks_board(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            products = root / "build/Pixhawk6X/bin"
            products.mkdir(parents=True)
            (products / "arducopter.apj").write_bytes(b"application")
            hwdef = root / "libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X"
            hwdef.mkdir(parents=True)
            (hwdef / "hwdef-bl.dat").write_text("# test\n")
            args = argparse.Namespace(boards_json='["Pixhawk6X"]', output=root / "out",
                                      version="v4.7.0-dev-1.0.0", commit="test", jobs=2, clean=False)
            calls = [None, None, None, None, subprocess.CalledProcessError(1, "bootloader")]
            with patch.object(build, "ROOT", root), patch.object(build, "run", side_effect=calls), \
                    patch.object(build, "radar_skip_reason", return_value=None), \
                    patch.object(build, "check_radar_elf"):
                self.assertEqual(build.build_batch(args), 1)
            result = json.loads((args.output / "results.json").read_text())["boards"][0]
            self.assertEqual(result["status"], "failed")

    def fixture(self, root):
        plan = {"version": "v4.7.0-dev-1.0.0", "commit": "test-commit",
                "candidates": ["Pixhawk6X", "small-board"], "excluded": []}
        build.write_json(root / "plan.json", plan)
        batch = root / "batches/radar-batch-0"
        batch.mkdir(parents=True)
        (batch / "Pixhawk6X_ArduCopter.apj").write_bytes(b"application")
        (batch / "Pixhawk6X_bootloader.bin").write_bytes(b"bootloader")
        result = {"version": plan["version"], "commit": plan["commit"], "boards": [
            {"board": "Pixhawk6X", "status": "built", "firmware": "Pixhawk6X_ArduCopter.apj",
             "bootloader": "Pixhawk6X_bootloader.bin"},
            {"board": "small-board", "status": "skipped", "reason": "HAL_RADARODOM_ENABLED=0"},
        ]}
        build.write_json(batch / "results.json", result)
        args = argparse.Namespace(plan=root / "plan.json", results=root / "batches", output=root / "assets",
                                  repository="example/ArduPilot", previous_tag="v4.7.0-dev-0.9.0")
        return args, batch / "results.json", result

    def test_collect_assets_and_checksums(self):
        with tempfile.TemporaryDirectory() as temp:
            args, _, _ = self.fixture(Path(temp))
            self.assertEqual(build.collect(args), 0)
            self.assertEqual((args.output / "RELEASE_NOTES.md").read_text(),
                             "**Full Changelog**: https://github.com/example/ArduPilot/compare/"
                             "v4.7.0-dev-0.9.0...v4.7.0-dev-1.0.0\n")
            self.assertEqual((args.output / "supported-boards.txt").read_text(), "Pixhawk6X\n")
            self.assertTrue((args.output / "Pixhawk6X_ArduCopter.apj").is_file())
            self.assertTrue((args.output / "Pixhawk6X_bootloader.bin").is_file())
            self.assertFalse(any("v4.7.0" in item.name for item in args.output.iterdir()))
            for line in (args.output / "SHA256SUMS").read_text().splitlines():
                digest, name = line.split("  ", 1)
                self.assertEqual(digest, hashlib.sha256((args.output / name).read_bytes()).hexdigest())

    def test_first_release_changelog(self):
        with tempfile.TemporaryDirectory() as temp:
            args, _, _ = self.fixture(Path(temp))
            args.previous_tag = None
            self.assertEqual(build.collect(args), 0)
            self.assertEqual((args.output / "RELEASE_NOTES.md").read_text(),
                             "**Full Changelog**: https://github.com/example/ArduPilot/commits/v4.7.0-dev-1.0.0\n")

    def test_collect_rejects_incomplete_failed_or_wrong_commit(self):
        for failure in ("missing", "failed", "commit", "duplicate", "asset", "no-firmware"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as temp:
                args, path, result = self.fixture(Path(temp))
                if failure == "missing":
                    result["boards"].pop()
                elif failure == "failed":
                    result["boards"][1]["status"] = "failed"
                elif failure == "commit":
                    result["commit"] = "wrong"
                elif failure == "duplicate":
                    result["boards"].append(result["boards"][0])
                elif failure == "asset":
                    result["boards"][0]["firmware"] = "../wrong.apj"
                else:
                    (path.parent / "Pixhawk6X_ArduCopter.apj").unlink()
                build.write_json(path, result)
                with self.assertRaises(ValueError):
                    build.collect(args)


if __name__ == "__main__":
    unittest.main()
