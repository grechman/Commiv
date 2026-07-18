#!/bin/bash
# ABI symbol check (docs/abi.md): the shared library's exported commiv_*
# symbols must be a superset of the committed baseline — a REMOVED symbol is
# an ABI break and fails; a NEW symbol requires updating the baseline in the
# same commit (run with --update after a deliberate, reviewed addition).
#
# Usage: zig build lib -Doptimize=ReleaseFast && bash tools/abi_check.sh
set -euo pipefail
cd "$(dirname "$0")/.."
LIB=zig-out/lib/libcommiv.so
BASE=tools/abi_symbols.txt
[ -f "$LIB" ] || { echo "build the lib first: zig build lib -Doptimize=ReleaseFast"; exit 1; }

current=$(nm -D --defined-only "$LIB" | awk '$3 ~ /^commiv_/ {print $3}' | sort)

if [ "${1:-}" = "--update" ]; then
    echo "$current" > "$BASE"
    echo "baseline updated: $(echo "$current" | wc -l) symbols"
    exit 0
fi

[ -f "$BASE" ] || { echo "no baseline; run: bash tools/abi_check.sh --update"; exit 1; }

removed=$(comm -23 "$BASE" <(echo "$current"))
added=$(comm -13 "$BASE" <(echo "$current"))

if [ -n "$removed" ]; then
    echo "ABI BREAK — exported symbols removed:"
    echo "$removed"
    exit 1
fi
if [ -n "$added" ]; then
    echo "New exported symbols (update the baseline in this commit if deliberate):"
    echo "$added"
    echo "run: bash tools/abi_check.sh --update"
    exit 1
fi
echo "ABI OK: $(wc -l < "$BASE") symbols, no drift"
