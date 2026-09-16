# Windows counterpart of scripts/julia_rb5.sh.
#   powershell -ExecutionPolicy Bypass -File scripts\julia_rb5.ps1 scripts\calibrate_rb5.jl ...
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$julia = $env:RWHE_JULIA
if (-not $julia) {
    $found = Get-Command julia -ErrorAction SilentlyContinue
    if (-not $found) {
        Write-Error "Julia is not on PATH. Install it, or set RWHE_JULIA to the executable."
    }
    $julia = $found.Source
}
& $julia --startup-file=no --project="$root\environments\rb5" @args
exit $LASTEXITCODE
