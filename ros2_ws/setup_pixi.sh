#!/usr/bin/env bash
# Run at container startup, as the configured container user.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
workspace=$(pwd -P)
environment=${1:-default}
case "$environment" in default|mac-cpu|l4t) ;; *) exit 2 ;; esac
prefix="$workspace/.pixi/envs/$environment"

# Prevent two copies of this startup script from installing simultaneously.
exec 9< setup_pixi.sh
flock 9

test -f pixi.toml && test -f pixi.lock
command -v pixi >/dev/null

for path in .pixi .pixi/envs "$prefix"; do
    if [ -L "$path" ]; then
        echo "Stop: symlinked Pixi environments need manual setup: $path" >&2
        exit 1
    fi
done

launchers_match() {
    local tool
    for tool in colcon register-python-argcomplete ros2; do
        [ -f "$prefix/bin/$tool" ] || return 1
        head -n 3 "$prefix/bin/$tool" |
            grep -Fq "$prefix/bin/python" || return 1
    done
}

repair=false
if [ -d .pixi ]; then
    if ! blocked=$(find .pixi \( -type d -o -name '*lock*' \) ! -writable -print -quit); then
        repair=true
    elif [ -n "$blocked" ]; then
        repair=true
    fi

    if [ -d "$prefix" ] && ! launchers_match; then
        repair=true
    fi
fi

if "$repair"; then
    # Keep the backup beside the original directory.
    backup=$(mktemp -d "$workspace/.pixi-backup.XXXXXX")
    mv -T -- .pixi "$backup"
    printf 'Previous Pixi environment saved at: %s\n' "$backup"

    mkdir .pixi
    if [ -f "$backup/config.toml" ]; then
        cp -- "$backup/config.toml" .pixi/config.toml
    fi
fi

rm -f -- "$prefix/.rover-ready"
pixi install --locked --environment "$environment"

launchers_match
PATH="$prefix/bin:$PATH" timeout 30 "$prefix/bin/colcon" --help >/dev/null
PATH="$prefix/bin:$PATH" timeout 30 "$prefix/bin/register-python-argcomplete" colcon >/dev/null
test -f "$prefix/setup.bash"
test -f "$prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash"

printf '%s\n' "$prefix" > "$prefix/.rover-ready"
printf 'Pixi and colcon ready: %s\n' "$environment"
