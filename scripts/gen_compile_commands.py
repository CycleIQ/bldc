#!/usr/bin/env python3
"""Generate compile_commands.json from the firmware make rules."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx")


def discover_boards(root):
    boards = []
    for header in root.glob("hwconf/**/hw_*.h"):
        if header.name.endswith("_core.h"):
            continue
        boards.append(header.stem.removeprefix("hw_"))
    return sorted(set(boards))


def run(cmd, root):
    return subprocess.run(
        cmd,
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def find_firmware_make_command(make_output):
    for line in make_output.splitlines():
        if "make/fw.mk" not in line:
            continue
        try:
            argv = shlex.split(line)
        except ValueError:
            continue
        if "-f" in argv and any(arg.endswith("make/fw.mk") for arg in argv):
            return argv
    return None


def force_verbose_dry_run(argv):
    cmd = [argv[0], "-B", "-n"] + argv[1:]
    return [
        "USE_VERBOSE_COMPILE=yes" if arg == "USE_VERBOSE_COMPILE=no" else arg
        for arg in cmd
    ]


def is_compile_command(argv):
    if "-c" not in argv:
        return False
    compiler = Path(argv[0]).name
    return compiler in {
        "cc",
        "clang",
        "clang++",
        "gcc",
        "g++",
        "arm-none-eabi-gcc",
        "arm-none-eabi-g++",
    }


def find_source(argv, root):
    output_index = argv.index("-o") if "-o" in argv else len(argv)
    for token in reversed(argv[1:output_index]):
        if token.startswith("-") or not token.endswith(SOURCE_SUFFIXES):
            continue
        source = (root / token).resolve() if not os.path.isabs(token) else Path(token)
        if source.exists():
            return source
    return None


def parse_compile_commands(make_output, root):
    entries = {}
    for line in make_output.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            argv = shlex.split(line)
        except ValueError:
            continue
        if not argv or not is_compile_command(argv):
            continue
        source = find_source(argv, root)
        if source is None:
            continue
        entries[str(source)] = {
            "directory": str(root),
            "file": str(source),
            "arguments": argv,
        }
    return [entries[path] for path in sorted(entries)]


def main():
    root = Path(__file__).resolve().parents[1]
    boards = discover_boards(root)

    parser = argparse.ArgumentParser(
        description="Generate compile_commands.json for clangd from make output."
    )
    parser.add_argument(
        "--board",
        default=boards[0] if boards else None,
        help="Firmware board target to model. Defaults to the first discovered board.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "compile_commands.json",
        help="Path to write. Defaults to ./compile_commands.json.",
    )
    args = parser.parse_args()

    if not args.board:
        print("error: no board specified and no hwconf boards were found", file=sys.stderr)
        return 2

    top = run(["make", "-n", args.board, "V=1"], root)
    firmware_make = find_firmware_make_command(top.stdout)
    if firmware_make is None:
        print(top.stdout, file=sys.stderr)
        print(f"error: could not find firmware make command for {args.board}", file=sys.stderr)
        return 2

    compile_run = run(force_verbose_dry_run(firmware_make), root)
    if compile_run.returncode != 0:
        print(compile_run.stdout, file=sys.stderr)
        return compile_run.returncode

    entries = parse_compile_commands(compile_run.stdout, root)
    if not entries:
        print("error: no C/C++ compile commands were found", file=sys.stderr)
        return 2

    output = args.output if args.output.is_absolute() else root / args.output
    output.write_text(json.dumps(entries, indent=2) + "\n")
    print(f"wrote {len(entries)} entries to {output} for board {args.board}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
