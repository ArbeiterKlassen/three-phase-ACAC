$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$tempDir = Join-Path ([System.IO.Path]::GetTempPath()) "acdc_halfbridge_f407_tests"
$testExe = Join-Path $tempDir "test_acdc.exe"
$lockTestExe = Join-Path $tempDir "test_commissioning_lock.exe"

New-Item -ItemType Directory -Force -Path $tempDir | Out-Null

& gcc -std=c11 -O2 -Wall -Wextra -Wpedantic `
    -DACDC_HARDWARE_COMMISSIONED=1 `
    -I (Join-Path $repoRoot "App\Inc") `
    (Join-Path $PSScriptRoot "test_acdc.c") `
    (Join-Path $repoRoot "App\Src\acdc_measurement.c") `
    (Join-Path $repoRoot "App\Src\acdc_control.c") `
    (Join-Path $repoRoot "App\Src\acdc_application.c") `
    -lm -o $testExe
if ($LASTEXITCODE -ne 0) {
    throw "Host test build failed."
}

& $testExe
if ($LASTEXITCODE -ne 0) {
    throw "Host tests failed."
}

& gcc -std=c11 -O2 -Wall -Wextra -Wpedantic `
    -DACDC_HARDWARE_COMMISSIONED=0 `
    -I (Join-Path $repoRoot "App\Inc") `
    (Join-Path $PSScriptRoot "test_commissioning_lock.c") `
    (Join-Path $repoRoot "App\Src\acdc_measurement.c") `
    (Join-Path $repoRoot "App\Src\acdc_control.c") `
    (Join-Path $repoRoot "App\Src\acdc_application.c") `
    -lm -o $lockTestExe
if ($LASTEXITCODE -ne 0) {
    throw "Commissioning-lock test build failed."
}

& $lockTestExe
if ($LASTEXITCODE -ne 0) {
    throw "Commissioning-lock test failed."
}
