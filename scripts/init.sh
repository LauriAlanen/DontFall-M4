#!/usr/bin/env bash
# One-time (and after west.yml changes) setup of the M4 west workspace:
# builds the Zephyr container, then `west init -l` + `west update` into the
# parent directory of this repo (the workspace, e.g. ~/Projects/DontFall-M4-ws).
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

echo "==> Building container dontfall-zephyr (UID=$UID GID=$GID)"
docker compose -f "$REPO/docker-compose.yml" build

if [ ! -d "$WEST_TOPDIR/.west" ]; then
    echo "==> west init -l (workspace: $WEST_TOPDIR)"
    zrun west init -l "$REPO"
fi

echo "==> west update"
zrun west update --narrow -o=--depth=1
echo "==> Done. Build with scripts/build.sh"
