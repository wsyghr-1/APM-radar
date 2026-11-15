#!/usr/bin/env python3
"""Plan a tag-pinned ChibiOS ArduCopter radar firmware release."""

import argparse
import json
from pathlib import Path
import re

from board_list import BoardList


ROOT = Path(__file__).resolve().parents[2]
VERSION_PATTERN = r"v4\.7\.0-dev-(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"


def validate_version(version):
    if re.fullmatch(VERSION_PATTERN, version) is None:
        raise ValueError("Expected a version such as v4.7.0-dev-1.0.0")
    return version


def validate_board(board):
    if re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+-]*", board) is None:
        raise ValueError(f"Invalid board name: {board!r}")
    if not (ROOT / "libraries/AP_HAL_ChibiOS/hwdef" / board / "hwdef.dat").is_file():
        raise ValueError(f"Not a ChibiOS board: {board}")
    return board


def make_plan(version, commit, batch_count):
    validate_version(version)
    if not 1 <= batch_count <= 32:
        raise ValueError("Batch count must be between 1 and 32")
    inventory = BoardList()
    definitions = {board.name: board for board in inventory.boards}
    candidates = []
    excluded = []
    for name in inventory.find_autobuild_boards("Copter"):
        if name.lower().endswith("-bdshot"):
            excluded.append({"board": name, "reason": "bdshot variants are not published"})
            continue
        hwdef = ROOT / "libraries/AP_HAL_ChibiOS/hwdef" / name / "hwdef.dat"
        if not hwdef.is_file() or definitions[name].toolchain != "arm-none-eabi":
            excluded.append({"board": name, "reason": "outside ChibiOS arm-none-eabi release scope"})
            continue
        candidates.append(validate_board(name))
    if not candidates:
        raise ValueError("No candidate boards found")
    count = min(batch_count, len(candidates))
    batches = [candidates[index::count] for index in range(count)]
    return {
        "version": version,
        "commit": commit,
        "candidates": candidates,
        "excluded": excluded,
        "matrix": {"include": [
            {"batch": index, "boards": boards} for index, boards in enumerate(batches)
        ]},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--batches", type=int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--github-output", type=Path)
    args = parser.parse_args()
    plan = make_plan(args.version, args.commit, args.batches)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")
    if args.github_output:
        with args.github_output.open("a", encoding="utf-8") as stream:
            stream.write("matrix=" + json.dumps(plan["matrix"], separators=(",", ":")) + "\n")
    print(f"{len(plan['candidates'])} candidates in {len(plan['matrix']['include'])} batches")


if __name__ == "__main__":
    main()
