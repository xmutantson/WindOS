#!/usr/bin/env python3
"""Compile and run offline C++ regression tests with mocked peripherals.

Requires Python 3.8+ and an existing C++11 compiler. No downloads, serial ports,
Arduino installation, or hardware operations. Build files live in an owned
temporary directory and are removed after execution.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def sketch_translation_unit(root, include_tests):
    source = (root / "Main.ino").read_text(encoding="utf-8")
    # Preserve strings while removing comments, then generate Arduino-style
    # forward declarations for this sketch's ordinary free functions.
    tokens = r'("(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\')|(/\*.*?\*/|//[^\n]*)'
    uncommented = re.sub(tokens, lambda match: match.group(1) or "", source, flags=re.S)
    functions = re.findall(
        r"^\s*((?:const\s+)?(?:void|bool|float|double|long|int|int8_t|uint8_t|uint32_t|char)"
        r"\s*\*?\s+\w+\s*\([^{};]*?\))\s*\{", uncommented, flags=re.M
    )
    if not functions:
        raise RuntimeError("No sketch function declarations found")
    lines = ['#include "MockHardware.h"', '#include "WindOSCore.h"']
    lines.extend(signature + ";" for signature in functions)
    lines.append("#include " + json.dumps((root / "Main.ino").as_posix()))
    if include_tests:
        lines.append("#include " + json.dumps((root / "tests/sketch_tests.cpp").as_posix()))
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"),
                        help="existing compiler executable (default: CXX or g++)")
    parser.add_argument("--syntax-only", action="store_true",
                        help="compile the complete mocked sketch without running tests")
    args = parser.parse_args()
    compiler = shutil.which(args.cxx)
    if not compiler:
        raise SystemExit("C++ compiler not found: " + args.cxx)
    # Select runtime DLLs from the same toolchain on Windows (e.g. MinGW),
    # ahead of unrelated Python-distribution DLLs already present in PATH.
    environment = os.environ.copy()
    environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
    root = Path(__file__).resolve().parents[1]
    flags = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror=return-type",
             "-Wno-unused-variable", "-Wno-sign-compare", "-Wno-parentheses",
             "-I" + str(root / "tests/host_stubs"), "-I" + str(root)]
    with tempfile.TemporaryDirectory(prefix="windos-tests-") as directory:
        build = Path(directory)
        sketch = build / "sketch.cpp"
        sketch.write_text(sketch_translation_unit(root, not args.syntax_only), encoding="utf-8")
        if args.syntax_only:
            subprocess.run(flags + ["-fsyntax-only", str(sketch)], check=True, timeout=90, env=environment)
            print("PASS: complete sketch host syntax check (mock hardware)")
            return
        for name, source in [("core", root / "tests/core_tests.cpp"), ("sketch", sketch)]:
            if not source.is_file():
                raise SystemExit("Missing test source: " + str(source))
            executable = build / (name + (".exe" if os.name == "nt" else ""))
            subprocess.run(flags + [str(source), "-o", str(executable)], check=True, timeout=90, env=environment)
            cases = ([None] if name == "core" else
                     subprocess.check_output([str(executable), "--list"],
                                             text=True, timeout=20, env=environment).splitlines())
            if not cases:
                raise RuntimeError("No sketch behavior cases registered")
            for case in cases:
                subprocess.run([str(executable)] + ([case] if case else []),
                               check=True, timeout=20, env=environment)
            print("PASS: " + name + " regression executable (" + str(len(cases)) +
                  " cases)", flush=True)
    print("PASS: all offline tests; no hardware or network accessed")


if __name__ == "__main__":
    main()
