#!/usr/bin/env bash
set -euo pipefail
rwhe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rwhe_julia="${RWHE_JULIA:-$HOME/.local/share/rwhe/julia-1.10.10/bin/julia}"
if [[ ! -x "$rwhe_julia" ]]; then
    echo "Julia is missing. Run bash scripts/install_rb5_local.sh first." >&2
    exit 1
fi
export JULIA_DEPOT_PATH="${RWHE_JULIA_DEPOT:-$HOME/.local/share/rwhe/julia-depot}"
exec "$rwhe_julia" --startup-file=no --project="$rwhe_root/environments/rb5" "$@"
