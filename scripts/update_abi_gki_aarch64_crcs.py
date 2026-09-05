#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
#
# Refresh CRC attributes in android/abi_gki_aarch64.xml from a freshly built
# vmlinux.symvers, so KMI validation passes after a stable merge or any change
# that legitimately recomputes genksyms CRCs for exported symbols.
#
# Scope of changes:
#   - Only the `crc` attribute on <elf-symbol> entries under
#     <elf-function-symbols> and <elf-variable-symbols> is rewritten.
#   - Symbols present in the XML but missing from vmlinux.symvers are reported
#     and left untouched.
#   - Symbols present in vmlinux.symvers but not in the XML are ignored
#     (the XML is the curated allow-list; we do not extend it here).
#   - All other XML structure (ordering, indentation around the changed
#     attribute, comments, namespace, root attributes, sibling elements) is
#     preserved as-is.
#
# Usage:
#   python3 scripts/update_abi_gki_aarch64_crcs.py \
#       [<abi_gki_aarch64.xml>] [<vmlinux.symvers>] [--dry-run]
#
# Defaults: android/abi_gki_aarch64.xml and out/vmlinux.symvers, relative to
# the repository root (i.e. the directory that contains this script's parent).
#
# Exit codes:
#   0  XML already matched, or was successfully updated.
#   1  XML differs from vmlinux.symvers and --dry-run was passed (drift only).
#   2  Argument / file error.
#
# After running this script, re-run KMI_function_symbols_test.py (or the
# project's ./build.sh) to confirm that no CRC mismatches remain.

from __future__ import annotations

import argparse
import os
import re
import sys
from typing import Final


CRC_RE: Final = re.compile(r"0x[0-9a-fA-F]+")
SCRIPT_DIR: Final = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT: Final = os.path.dirname(SCRIPT_DIR)


def normalise_crc(crc: str) -> str:
    """Return a canonical 0x + 8-hex-digit form for a CRC string.

    genksyms / modpost emit CRCs as `0x` followed by 1-8 hex digits with no
    leading zeros, while abi_gki_aarch64.xml stores them zero-padded to 8
    hex digits. We always emit the padded form so the XML output is stable.
    """
    if not CRC_RE.match(crc):
        raise ValueError(f"not a valid CRC: {crc!r}")
    value = int(crc[2:], 16)
    return "0x" + format(value, "08x")


def load_symvers(path: str) -> dict[str, str]:
    """Parse a vmlinux.symvers file into {symbol_name: canonical_crc}.

    vmlinux.symvers lines look like:
        <crc> <symbol> <module> <export-type> [<namespace>]
    Lines that don't match this layout are silently skipped.
    """
    out: dict[str, str] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 2:
                continue
            crc, name = parts[0], parts[1]
            if not CRC_RE.match(crc):
                continue
            out[name] = normalise_crc(crc)
    return out


# Match a single <elf-symbol .../> line and capture its name and crc attrs.
ELF_SYMBOL_RE: Final = re.compile(
    r"(<elf-symbol\b[^/>]*?\bname=')(?P<name>[^']+)"
    r"('[^/>]*?\bcrc=')(?P<crc>0x[0-9a-fA-F]+)(')",
)


def rewrite_xml(xml_text: str, symvers: dict[str, str]) -> tuple[str, int, int, list[str]]:
    """Rewrite CRC attributes in the XML text.

    Returns (new_text, changed, unchanged, missing_symbols).
    """
    changed = 0
    unchanged = 0
    missing: list[str] = []

    def repl(m: re.Match[str]) -> str:
        nonlocal changed, unchanged
        name = m.group("name")
        old_crc = normalise_crc(m.group("crc"))
        new_crc = symvers.get(name)
        if new_crc is None:
            missing.append(name)
            return m.group(0)
        if new_crc == old_crc:
            unchanged += 1
            return m.group(0)
        changed += 1
        return f"{m.group(1)}{name}{m.group(3)}{new_crc}{m.group(5)}"

    new_text = ELF_SYMBOL_RE.sub(repl, xml_text)
    return new_text, changed, unchanged, missing


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Refresh CRCs in android/abi_gki_aarch64.xml from vmlinux.symvers.",
    )
    parser.add_argument(
        "xml",
        nargs="?",
        default=os.path.join(REPO_ROOT, "android", "abi_gki_aarch64.xml"),
        help="Path to abi_gki_aarch64.xml (default: android/abi_gki_aarch64.xml)",
    )
    parser.add_argument(
        "symvers",
        nargs="?",
        default=os.path.join(REPO_ROOT, "out", "vmlinux.symvers"),
        help="Path to vmlinux.symvers (default: out/vmlinux.symvers)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would change but do not modify the XML.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-symbol missing-symbol warnings.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.xml):
        print(f"error: XML file not found: {args.xml}", file=sys.stderr)
        return 2
    if not os.path.isfile(args.symvers):
        print(f"error: symvers file not found: {args.symvers}", file=sys.stderr)
        return 2

    symvers = load_symvers(args.symvers)
    if not symvers:
        print(f"error: no CRCs parsed from {args.symvers}", file=sys.stderr)
        return 2

    with open(args.xml, "r", encoding="utf-8") as f:
        xml_text = f.read()

    new_text, changed, unchanged, missing = rewrite_xml(xml_text, symvers)

    print(f"Symbols matched in XML and symvers: {changed + unchanged}")
    print(f"  CRC already up-to-date: {unchanged}")
    print(f"  CRC needs refresh:      {changed}")
    if missing:
        print(f"  XML symbols missing from symvers: {len(missing)}")
        if not args.quiet:
            for name in missing[:20]:
                print(f"    - {name}")
            if len(missing) > 20:
                print(f"    ... and {len(missing) - 20} more")

    if changed == 0:
        print("No CRC drift detected. abi_gki_aarch64.xml already matches vmlinux.symvers.")
        return 0

    if args.dry_run:
        print(f"Dry-run: {changed} CRC(s) would be refreshed in {args.xml}.")
        return 1

    with open(args.xml, "w", encoding="utf-8") as f:
        f.write(new_text)
    print(f"Refreshed {changed} CRC(s) in {args.xml}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
