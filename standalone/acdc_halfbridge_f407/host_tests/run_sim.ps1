$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$tempDir = Join-Path ([System.IO.Path]::GetTempPath()) "acdc_halfbridge_f407_sim"
$exe = Join-Path $tempDir "sim_input_current.exe"

New-Item -ItemType Directory -Force -Path $tempDir | Out-Null

& gcc -std=c11 -O2 -Wall -Wextra -Wpedantic `
    (Join-Path $PSScriptRoot "sim_input_current.c") `
    -lm -o $exe
if ($LASTEXITCODE -ne 0) {
    throw "Simulation build failed."
}

foreach ($vac in @(31.0, 36.0, 41.0)) {
    Write-Host "=== Vac=$vac V RMS, load=1.8 A ==="
    & $exe $vac 1.8 10 2 --ilimit=5.5 | Select-Object -Last 7
    if ($LASTEXITCODE -ne 0) {
        throw "Simulation failed for Vac=$vac."
    }
}
