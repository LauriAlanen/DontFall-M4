#!/usr/bin/env bash
# Apply patches/<project>/*.patch to the west-managed projects (idempotent):
#   patches/zephyr/                   -> zephyr
#   patches/micro_ros_zephyr_module/  -> modules/lib/micro_ros_zephyr_module
# The DontFall Yocto recipe (dontfall-m4.bb) applies the zephyr ones, so dev
# and release build identical sources. Called by init.sh and build.sh.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

apply_dir() {
    local patches="$1" tree="$2"
    for p in "$patches"/*.patch; do
        [ -e "$p" ] || continue
        if git -C "$tree" apply --reverse --check "$p" 2>/dev/null; then
            continue    # already applied
        fi
        echo "==> applying $(basename "$p") to $(basename "$tree")"
        git -C "$tree" apply "$p"
    done
}

apply_dir "$REPO/patches/zephyr" "$REPO/../zephyr"
apply_dir "$REPO/patches/micro_ros_zephyr_module" "$REPO/../modules/lib/micro_ros_zephyr_module"
