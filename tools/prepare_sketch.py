#!/usr/bin/env python3
"""Stage the tracked firmware under an Arduino-compatible sketch directory.

Only copies files into this checkout's .build directory. It does not download,
compile, upload, or connect to a board. Requires Python 3.8 or newer.
"""

from pathlib import Path
import shutil


def main():
    root = Path(__file__).resolve().parents[1]
    build_root = root / ".build"
    sketch = build_root / "arduino" / "WindOS"
    sources = [(root / "Main.ino", "WindOS.ino"),
               (root / "WindOSCore.h", "WindOSCore.h")]
    for source, _ in sources:
        if not source.is_file():
            raise SystemExit("Missing firmware source: {}".format(source))
    # Refuse a redirected build directory rather than overwriting another path.
    expected = root / ".build" / "arduino" / "WindOS"
    if sketch.resolve() != expected:
        raise SystemExit("Refusing a redirected .build sketch directory")
    sketch.mkdir(parents=True, exist_ok=True)
    for source, name in sources:
        target = sketch / name
        if target.is_symlink():
            raise SystemExit("Refusing a symlinked staging file: {}".format(target))
        shutil.copyfile(source, target)
    print(sketch)
    print("Staged source only; no compilation or hardware operation performed.")


if __name__ == "__main__":
    main()
