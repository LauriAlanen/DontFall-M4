#!/usr/bin/env bash
# Load the freshly built M4 firmware on a running board via remoteproc.
#
#   scripts/deploy.sh <board-ip> [--log]
#
# Copies build/zephyr/zephyr.elf to /lib/firmware/dontfall-m4.elf (the same
# name the image ships, so it overrides the packaged firmware until the next
# image deploy), restarts the M4 and, with --log, prints its RAM console
# (remoteproc trace0).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF="$REPO/build/zephyr/zephyr.elf"
FW=dontfall-m4.elf

SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o ConnectTimeout=5 -o NumberOfPasswordPrompts=0 -o LogLevel=ERROR)

die() { echo "deploy.sh: $*" >&2; exit 1; }

TARGET=""; LOG=0
for a in "$@"; do
    case "$a" in
        --log)      LOG=1 ;;
        -h|--help)  sed -n '2,9p' "$0"; exit 0 ;;
        -*)         die "unknown option: $a" ;;
        *)          [ -z "$TARGET" ] || die "only one board ip"; TARGET=$a ;;
    esac
done
[ -n "$TARGET" ] || die "usage: scripts/deploy.sh <board-ip> [--log]"
[ -f "$ELF" ] || die "no $ELF - run scripts/build.sh first"

echo "==> scp zephyr.elf -> root@$TARGET:/lib/firmware/$FW"
scp -q "${SSH_OPTS[@]}" "$ELF" "root@$TARGET:/lib/firmware/$FW"

echo "==> restarting M4"
ssh "${SSH_OPTS[@]}" "root@$TARGET" "
    R=/sys/class/remoteproc/remoteproc0
    [ \"\$(cat \$R/state)\" = offline ] || echo stop > \$R/state
    echo -n $FW > \$R/firmware
    echo start > \$R/state
    echo \"state: \$(cat \$R/state)\"
"

if [ "$LOG" = 1 ]; then
    sleep 1
    echo "==> M4 log (trace0)"
    ssh "${SSH_OPTS[@]}" "root@$TARGET" \
        'mountpoint -q /sys/kernel/debug || mount -t debugfs none /sys/kernel/debug;
         cat /sys/kernel/debug/remoteproc/remoteproc0/trace0'
fi
