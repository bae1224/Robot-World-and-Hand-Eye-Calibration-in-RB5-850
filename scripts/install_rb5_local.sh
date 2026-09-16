#!/usr/bin/env bash
# User-local installation, no venv, sudo, apt, shell-profile edits or robot access.
set -euo pipefail
rwhe_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$rwhe_root"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
    echo "This installer targets Ubuntu Linux x86_64." >&2
    exit 1
fi
python3 -c 'import sys; assert sys.version_info[:2] == (3,10), "Use Ubuntu 22.04 Python 3.10"; assert sys.prefix == sys.base_prefix, "Run outside a virtual environment"'
rwhe_downloads="$rwhe_root/.runtime/downloads"
mkdir -p "$rwhe_downloads"
if ! python3 -m pip --version >/dev/null 2>&1; then
    curl --fail --location --retry 3 https://bootstrap.pypa.io/get-pip.py -o "$rwhe_downloads/get-pip.py"
    python3 "$rwhe_downloads/get-pip.py" --user
fi
python3 -m pip install --user --only-binary=:all: -r hardware/requirements-rb5.txt
rwhe_julia_parent="$HOME/.local/share/rwhe"
if [[ ! -x "$rwhe_julia_parent/julia-1.10.10/bin/julia" ]]; then
    curl --fail --location --retry 3 \
        https://julialang-s3.julialang.org/bin/linux/x64/1.10/julia-1.10.10-linux-x86_64.tar.gz \
        -o "$rwhe_downloads/julia.tar.gz"
    python3 - "$rwhe_downloads/julia.tar.gz" <<'PY'
import hashlib
import sys
with open(sys.argv[1], 'rb') as source:
    digest = hashlib.file_digest(source, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else hashlib.sha256(source.read()).hexdigest()
if digest != '6a78a03a71c7ab792e8673dc5cedb918e037f081ceb58b50971dfb7c64c5bf81':
    raise SystemExit('Julia archive SHA256 mismatch')
PY
    mkdir -p "$rwhe_julia_parent"
    tar -xzf "$rwhe_downloads/julia.tar.gz" -C "$rwhe_julia_parent"
fi
bash scripts/julia_rb5.sh -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()'
python3 -m unittest discover -s tests -v
bash scripts/julia_rb5.sh tests/rb5_solver.jl
echo "Installed Python packages under the user site and Julia under ~/.local/share/rwhe."
echo "Run: python3 -m hardware.rb5_camera doctor"
