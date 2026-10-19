#!/usr/bin/env bash
# Host tests: ztest suites under tests/, built for native_sim and run by
# twister. Results in build/twister/. Extra args go to twister
# (e.g. scripts/test.sh -s dontfall.tilt   for one test).
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

zrun west twister -T tests -p native_sim/native/64 -O build/twister --inline-logs "$@"
