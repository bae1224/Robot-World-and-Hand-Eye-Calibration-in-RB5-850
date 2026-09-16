#!/usr/bin/env bash
# Prefers the pinned local install from scripts/install_rb5_local.sh, and falls
# back to whatever `julia` is on PATH so the same command works on a machine
# (Windows, or a shared workstation) where that install was never run.
set -euo pipefail
rwhe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rwhe_julia="${RWHE_JULIA:-$HOME/.local/share/rwhe/julia-1.10.10/bin/julia}"
rwhe_env="$rwhe_root/environments/rb5"
if [[ -x "$rwhe_julia" ]]; then
    export JULIA_DEPOT_PATH="${RWHE_JULIA_DEPOT:-$HOME/.local/share/rwhe/julia-depot}"
elif command -v julia >/dev/null 2>&1; then
    rwhe_julia="$(command -v julia)"
    # environments/rb5 pins a Manifest resolved against Julia 1.10, which will not
    # precompile on a newer one; the portable environment resolves itself instead.
    rwhe_env="$rwhe_root/environments/rb5-portable"
    echo "Using julia from PATH: $rwhe_julia (environments/rb5-portable)" >&2
    echo "Run scripts/install_rb5_local.sh to reproduce recorded results exactly." >&2
    "$rwhe_julia" --startup-file=no --project="$rwhe_env"         -e 'using Pkg; isfile(joinpath(dirname(Base.active_project()), "Manifest.toml")) || Pkg.instantiate()'
else
    echo "Julia is missing. Run bash scripts/install_rb5_local.sh first," >&2
    echo "or install Julia and make sure it is on PATH." >&2
    exit 1
fi
exec "$rwhe_julia" --startup-file=no --project="$rwhe_env" "$@"
