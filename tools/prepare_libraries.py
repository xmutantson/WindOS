#!/usr/bin/env python3
"""Verify and stage pinned libraries in .build; downloads require --download.

No toolchain install, global library install, board discovery, or upload.
Requires Python 3.8+; uses only the standard library.
"""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true',
                        help='allow downloading missing pinned archives from Arduino')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / '.build'
    archives = build / 'downloads'
    libraries = build / 'libraries'
    for directory in (build, archives, libraries):
        if directory.resolve() != directory:
            raise SystemExit('Refusing redirected build directory: ' + str(directory))
        directory.mkdir(parents=True, exist_ok=True)
    entries = json.loads((root / 'tools/arduino-libraries.lock.json').read_text(encoding='utf-8'))
    for entry in entries:
        url = entry['url']
        if not url.startswith('https://downloads.arduino.cc/libraries/'):
            raise SystemExit('Unexpected library host: ' + url)
        archive_path = archives / url.rsplit('/', 1)[1]
        if archive_path.is_symlink():
            raise SystemExit('Refusing symlinked archive: ' + str(archive_path))
        if archive_path.is_file():
            data = archive_path.read_bytes()
        elif args.download:
            with urllib.request.urlopen(url, timeout=45) as response:
                data = response.read()
        else:
            raise SystemExit('Missing archive: {} (use --download to fetch)'.format(archive_path))
        if hashlib.sha256(data).hexdigest() != entry['sha256']:
            raise SystemExit('Checksum mismatch: ' + str(archive_path))
        if not archive_path.exists():
            archive_path.write_bytes(data)
        with zipfile.ZipFile(archive_path) as archive:
            for member in archive.infolist():
                target = libraries / member.filename
                if libraries not in target.resolve().parents or target.resolve() != target:
                    raise SystemExit('Refusing redirected archive member: ' + member.filename)
            archive.extractall(libraries)
        print('Verified and staged: {} {}'.format(entry['name'], entry['version']))
    print('Library directory: ' + str(libraries))


if __name__ == '__main__':
    main()
