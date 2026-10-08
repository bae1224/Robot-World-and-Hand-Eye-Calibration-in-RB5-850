#!/usr/bin/env bash
# Install only the pinned Julia runtime and RWHE solver environment.
# No sudo, robot connection, Python package installation or shell-profile changes.
set -euo pipefail
rwhe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rwhe_julia_parent="$HOME/.local/share/rwhe"
rwhe_default_julia="$rwhe_julia_parent/julia-1.10.10/bin/julia"
rwhe_julia="${RWHE_JULIA:-$rwhe_default_julia}"
rwhe_downloads="$rwhe_root/.runtime/downloads"
rwhe_archive="$rwhe_downloads/julia-1.10.10-linux-x86_64.tar.gz"
rwhe_sha256='6a78a03a71c7ab792e8673dc5cedb918e037f081ceb58b50971dfb7c64c5bf81'

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
    echo 'This installer targets Linux x86_64.' >&2
    exit 1
fi
if [[ ! -x "$rwhe_julia" ]]; then
    if [[ "$rwhe_julia" != "$rwhe_default_julia" ]]; then
        echo 'RWHE_JULIA points to a missing executable. Unset it to install the default runtime.' >&2
        exit 1
    fi
    mkdir -p "$rwhe_downloads" "$rwhe_julia_parent"
    if ! { [[ -f "$rwhe_archive" ]] && echo "$rwhe_sha256  $rwhe_archive" | sha256sum --check --status; }; then
        rwhe_temp="$(mktemp "$rwhe_downloads/julia-download.XXXXXX")"
        trap 'rm -f -- "$rwhe_temp"' EXIT
        curl --fail --location --retry 3 --connect-timeout 20 --max-time 900 \
          https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.10-linux-x86_64.tar.gz \
          --output "$rwhe_temp"
        echo "$rwhe_sha256  $rwhe_temp" | sha256sum --check
        mv -- "$rwhe_temp" "$rwhe_archive"
        trap - EXIT
    fi
    tar -xzf "$rwhe_archive" -C "$rwhe_julia_parent"
fi
"$rwhe_julia" --startup-file=no --version
export JULIA_NUM_PRECOMPILE_TASKS="${JULIA_NUM_PRECOMPILE_TASKS:-2}"
bash "$rwhe_root/scripts/julia_rb5.sh" -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()'
echo 'Julia runtime and environments/rb5 dependencies are ready.'
echo 'Validate with: bash scripts/julia_rb5.sh tests/rb5_solver.jl'
