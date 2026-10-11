#!/usr/bin/env bash
# Verify west.yml pins the same Zephyr commit that meta-zephyr builds in the
# DontFall Yocto image (../../DontFall/meta-zephyr, if checked out).
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
inc="$repo/../../DontFall/meta-zephyr/meta-zephyr-core/recipes-kernel/zephyr-kernel/zephyr-kernel-src-3.7.0.inc"

west_rev=$(awk '/- name: zephyr$/ {z=1} z && /revision:/ {print $2; exit}' "$repo/west.yml")
echo "west.yml zephyr:   $west_rev"

if [ ! -f "$inc" ]; then
    echo "meta-zephyr not present ($inc) - nothing to compare"
    exit 0
fi

yocto_rev=$(sed -n 's/^SRCREV_default = "\(.*\)"/\1/p' "$inc")
echo "meta-zephyr zephyr: $yocto_rev"

if [ "$west_rev" != "$yocto_rev" ]; then
    echo "MISMATCH: update west.yml or meta-zephyr so dev and release build the same Zephyr" >&2
    exit 1
fi
echo "OK: pins match"
