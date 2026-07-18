#!/bin/bash
# Build pip-installable platform wheels for the commiv Python binding.
#
# Each wheel is pure-Python (ctypes) plus ONE prebuilt libcommiv for its
# platform, cross-compiled by Zig — so `pip install commiv` needs no
# toolchain. The library is libc-free on Linux (no GLIBC version deps), so
# the manylinux2014 tag is safe on any distro pip accepts it on; macOS
# builds link libSystem (mandatory on that OS).
#
# Usage: bash tools/build_wheels.sh          (from the repo root)
# Output: dist/commiv-<ver>-py3-none-<plat>.whl  (one per platform)
#
# Targets: linux x86_64 + macOS arm64 + macOS x86_64. The macOS wheels are
# cross-compiled and NOT runtime-tested here (no Apple hardware in the loop)
# — say so honestly wherever they are published until someone runs
# test_smoke.py on a Mac.
set -euo pipefail
cd "$(dirname "$0")/.."
PKG=bindings/python
DIST="$PWD/dist"
mkdir -p "$DIST"
rm -f "$PKG"/commiv/libcommiv.so "$PKG"/commiv/libcommiv.dylib

build_one() { # <zig-target> <wheel-plat-tag> <libname>
    local target="$1" plat="$2" libname="$3"
    echo "=== $target -> $plat"
    zig build lib -Dtarget="$target" -Doptimize=ReleaseFast
    cp "zig-out/lib/$libname" "$PKG/commiv/$libname"
    local tmp
    tmp=$(mktemp -d)
    (cd "$PKG" && python3 -m pip wheel . --no-deps --no-build-isolation -w "$tmp" -q)
    python3 -m wheel tags --remove --platform-tag="$plat" "$tmp"/commiv-*-py3-none-any.whl >/dev/null
    mv "$tmp"/commiv-*.whl "$DIST/"
    rm -rf "$tmp" "$PKG/commiv/$libname"
}

build_one x86_64-linux-gnu.2.17 manylinux2014_x86_64 libcommiv.so
build_one aarch64-macos macosx_11_0_arm64 libcommiv.dylib
build_one x86_64-macos macosx_10_15_x86_64 libcommiv.dylib

echo "=== wheels:"
ls -la "$DIST"/commiv-*.whl
echo "Upload: python3 -m twine upload dist/commiv-*.whl  (needs PyPI token)"
