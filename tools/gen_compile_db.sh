#!/usr/bin/env bash
# Regenerate every compile_commands.json in the tree, so clangd resolves each
# subproject with its REAL compiler flags.
#
#   tools/gen_compile_db.sh
#
# Why this exists: clangd finds a compilation database by walking UP from the
# file being edited and checking each ancestor directory (and each ancestor's
# `build/` subdir), stopping at the first database it finds. With no database it
# falls back to parsing with no flags at all, which is why an unconfigured tree
# reports nonsense like "'config.h' file not found", "unknown type name
# 'DShotRMT'", or "C does not support default arguments" on dshot.h — none of
# which are real. The fix is simply to make sure a database exists at the root of
# every subproject; no .clangd flag-munging is needed (verified: 0 errors across
# brain, receiver, core and replay with databases alone).
#
# PlatformIO projects need this re-run whenever platformio.ini changes (new env,
# changed build_flags, bumped platform) — the database captures flags, not intent.
# Databases are gitignored: they contain absolute paths to this machine's
# toolchain and are worthless to anyone else.
#
# Every PlatformIO project is discovered automatically, so a new subproject is
# picked up with no edit here.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIO="${PIO:-$HOME/.platformio/penv/bin/pio}"
rc=0

if [ ! -x "$PIO" ]; then
    echo "!! platformio not found at $PIO (override with PIO=/path/to/pio)" >&2
    echo "   skipping PlatformIO projects; CMake projects below still run." >&2
else
    # -not -path "*/.pio/*" keeps us out of PlatformIO's own build tree, which
    # contains platformio.ini files belonging to downloaded library examples.
    while IFS= read -r ini; do
        proj="$(dirname "$ini")"
        printf '==> pio  %s\n' "${proj#"$ROOT"/}"
        if ! (cd "$proj" && "$PIO" run -t compiledb); then
            echo "!! FAILED: ${proj#"$ROOT"/}" >&2
            rc=1
        fi
    done < <(find "$ROOT" -name platformio.ini -not -path "*/.pio/*" | sort)
fi

# CMake projects: configuring is enough, CMAKE_EXPORT_COMPILE_COMMANDS does the rest.
for cm in "$ROOT/tools/replay" "$ROOT/sunshine_core"; do
    [ -f "$cm/CMakeLists.txt" ] || continue
    printf '==> cmake %s\n' "${cm#"$ROOT"/}"
    if ! cmake -B "$cm/build" -S "$cm" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null; then
        echo "!! FAILED: ${cm#"$ROOT"/}" >&2
        rc=1
    fi
done

echo
echo "compilation databases now present:"
find "$ROOT" -name compile_commands.json -not -path "*/node_modules/*" -not -path "*/.pio/*" \
    | sed "s|$ROOT/|  |"

# Verify clangd actually resolves representative files, rather than assuming it.
if command -v clangd >/dev/null 2>&1; then
    echo
    echo "clangd check:"
    for f in sunshine_brain/src/dshot.cpp sunshine_receiver/src/main.cpp \
             sunshine_core/src/control.c tools/replay/replay.c; do
        [ -f "$ROOT/$f" ] || continue
        n=$(cd "$ROOT" && clangd --check="$f" 2>&1 | grep -c "error:")
        printf '  %-36s %s\n' "$f" "$([ "$n" -eq 0 ] && echo "ok" || echo "$n errors")"
        [ "$n" -eq 0 ] || rc=1
    done
fi

exit $rc
