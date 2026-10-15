#!/usr/bin/env bash
# Verify west.yml pins the same Zephyr commit that meta-zephyr builds in the
# DontFall Yocto image (../../DontFall/meta-zephyr, if checked out), and the
# same micro-ROS sources (micro_ros.yml + micro_ros_zephyr_module) that
# dontfall-m4.bb fetches (micro-ros-srcs.inc).
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# The version bitbake builds comes from the zephyr-m4 multiconfig, not the
# inc filename - derive it so this check keeps working across Zephyr bumps.
m4conf="$repo/../../DontFall/meta-df/meta-df-common/conf/multiconfig/zephyr-m4.conf"
ver=$(sed -n 's/^PREFERRED_VERSION_zephyr-kernel = "\(.*\)"$/\1/p' "$m4conf" 2>/dev/null)
if [ -z "$ver" ]; then
    echo "cannot read PREFERRED_VERSION_zephyr-kernel from $m4conf" >&2
    exit 1
fi
inc="$repo/../../DontFall/meta-zephyr/meta-zephyr-core/recipes-kernel/zephyr-kernel/zephyr-kernel-src-$ver.inc"

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

# micro-ROS: "name revision" pairs from both sides, sorted and compared.
srcs="$repo/../../DontFall/meta-df/meta-df-common/recipes-df/dontfall-m4/micro-ros-srcs.inc"
west_uros=$(cat "$repo/micro_ros.yml" "$repo/west.yml" | awk '
    /- name: / {name=$3}
    /revision:/ && name != "zephyr" {print name, $2}' | sort)
yocto_uros=$(sed -n 's/^SRCREV_\(.*\) = "\(.*\)"/\1 \2/p' "$srcs" | sort)
if [ "$west_uros" != "$yocto_uros" ]; then
    echo "MISMATCH: micro-ROS pins differ between west and $srcs:" >&2
    diff <(echo "$west_uros") <(echo "$yocto_uros") >&2 || true
    exit 1
fi
echo "micro-ROS: $(echo "$west_uros" | wc -l) pins"
echo "OK: pins match"
