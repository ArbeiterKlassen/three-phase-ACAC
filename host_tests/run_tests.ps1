$ErrorActionPreference = "Stop"

$testDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $testDir
$includeDir = Join-Path $root "App\Inc"
$testSource = Join-Path $testDir "test_main.c"
$sources = @(Get-ChildItem -LiteralPath (Join-Path $root "App\Src") -Filter "*.c" |
    Sort-Object Name |
    ForEach-Object { $_.FullName })
$buildDir = Join-Path $env:TEMP "bianliuqifuzainibian_host_tests"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if ($gcc) {
    $exe = Join-Path $buildDir "host_tests.exe"
    & $gcc.Source -std=c11 -Wall -Wextra -Werror "-I$includeDir" `
        $testSource $sources -lm -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    exit $LASTEXITCODE
}

$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "No GCC or Visual Studio C compiler was found."
}

$vsPath = & $vswhere -latest -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -property installationPath
if (-not $vsPath) {
    throw "The Visual Studio C/C++ toolset is not installed."
}

$devCmd = Join-Path $vsPath "Common7\Tools\VsDevCmd.bat"
$exe = Join-Path $buildDir "host_tests.exe"
$quotedSources = @($testSource) + $sources |
    ForEach-Object { '"' + $_ + '"' }
$compile = 'cl /nologo /std:c11 /utf-8 /W4 /WX /I"' + $includeDir +
    '" ' + ($quotedSources -join " ") + ' /Fe:"' + $exe + '"'
$command = 'call "' + $devCmd + '" -arch=x64 -host_arch=x64 >nul && ' +
    'cd /d "' + $buildDir + '" && ' + $compile + ' && "' + $exe + '"'

& cmd.exe /d /s /c $command
exit $LASTEXITCODE
