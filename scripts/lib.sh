# Shared helpers for scripts/*.sh - source, don't run.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEST_TOPDIR="$(cd "$REPO/.." && pwd)"
GID="${GID:-$(id -g)}"
# UID is a read-only bash variable; exporting is enough for compose.
export WEST_TOPDIR UID GID

# Run a command in the Zephyr container (no TTY when stdin isn't one).
zrun() {
    local args=(run --rm)
    [ -t 0 ] || args+=(-T)
    docker compose -f "$REPO/docker-compose.yml" "${args[@]}" zephyr "$@"
}
