#!/usr/bin/env bash
# Apply patches/zephyr/*.patch to the west-managed zephyr tree (idempotent).
# The DontFall Yocto recipe (dontfall-m4.bb) applies the same files, so dev
# and release build identical sources. Called by init.sh and build.sh.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ZEPHYR="$REPO/../zephyr"

for p in "$REPO"/patches/zephyr/*.patch; do
    [ -e "$p" ] || continue
    if git -C "$ZEPHYR" apply --reverse --check "$p" 2>/dev/null; then
        continue    # already applied
    fi
    echo "==> applying $(basename "$p") to zephyr"
    git -C "$ZEPHYR" apply "$p"
done
