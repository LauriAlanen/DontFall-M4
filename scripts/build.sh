#!/usr/bin/env bash
# Build the M4 firmware: build/zephyr/zephyr.elf. Extra args go to west build
# (e.g. scripts/build.sh -p always   for a pristine build).
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

"$REPO/scripts/apply-patches.sh"
zrun west build -b stm32mp157c_dk2 app -d build "$@"
