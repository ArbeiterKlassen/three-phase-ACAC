param(
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$source = Join-Path $PSScriptRoot "sim_acdc_halfbridge.c"
$tempDir = Join-Path ([System.IO.Path]::GetTempPath()) "acdc_halfbridge"
$exe = Join-Path $tempDir "sim_acdc_halfbridge.exe"
$summaryPath = Join-Path $repoRoot "docs\acdc_halfbridge_sweep.csv"
$plotScript = Join-Path $PSScriptRoot "plot_acdc_halfbridge.py"

New-Item -ItemType Directory -Force -Path $tempDir | Out-Null

if (-not $SkipBuild) {
    & gcc -std=c11 -O2 -Wall -Wextra -Wpedantic -I (Join-Path $repoRoot "App\Inc") `
        $source -lm -o $exe
    if ($LASTEXITCODE -ne 0) {
        throw "ACDC half-bridge simulation build failed."
    }
}

function Invoke-SimCase {
    param(
        [string]$Name,
        [double]$Vac,
        [double]$Load,
        [double]$Cf,
        [double]$Lf,
        [double]$ILimit,
        [string]$WavePath
    )

    $simArgs = @(
        "$Vac", "$Load", "$Cf", "$Lf",
        "--ilimit=$ILimit",
        "--stats-start=3.5",
        "--duration=3.9"
    )
    if ($WavePath) {
        $simArgs += "--csv=$WavePath"
    }
    $output = & $exe @simArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "Simulation case '$Name' failed: $($output -join [Environment]::NewLine)"
    }
    $resultLine = $output | Where-Object { $_ -match '^RESULT\s+' } | Select-Object -Last 1
    if (-not $resultLine) {
        throw "Simulation case '$Name' produced no RESULT line."
    }

    $record = [ordered]@{ case = $Name }
    foreach ($match in [regex]::Matches($resultLine, '([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)')) {
        $record[$match.Groups[1].Value] = [double]::Parse(
            $match.Groups[2].Value,
            [System.Globalization.CultureInfo]::InvariantCulture
        )
    }
    $format = "[{0}] Vac={1:F1}, load={2:F2} A, Lf={3:F2} mH, Cf={4:F1} uF, " +
        "limit={5:F2} A -> Vbus={6:F2} V, PF={7:F3}, THD={8:F1}%"
    Write-Host ($format -f $Name, $Vac, $Load, $Lf, $Cf, $ILimit,
        $record.vbus_mean, $record.pf, $record.thd)
    return [pscustomobject]$record
}

$rows = [System.Collections.Generic.List[object]]::new()

foreach ($vac in @(31.0, 36.0, 41.0)) {
    $rows.Add((Invoke-SimCase "input-${vac}-limit3p8" $vac 1.8 10 2 3.8 $null))
    $rows.Add((Invoke-SimCase "input-${vac}-limit5p5" $vac 1.8 10 2 5.5 $null))
}

foreach ($limit in @(3.8, 4.0, 4.3, 4.6, 5.0, 5.5)) {
    $rows.Add((Invoke-SimCase "limit-${limit}" 36 1.8 10 2 $limit $null))
}

foreach ($lf in @(0.5, 1.0, 2.0, 5.0)) {
    foreach ($cf in @(1.0, 4.7, 10.0, 22.0)) {
        $rows.Add((Invoke-SimCase "lc-L${lf}-C${cf}" 36 1.8 $cf $lf 5.5 $null))
    }
}

foreach ($load in @(0.5, 1.0, 1.8)) {
    $rows.Add((Invoke-SimCase "load-${load}" 36 $load 10 2 5.5 $null))
}

$wave38 = Join-Path $tempDir "reference_limit_3p8.csv"
$wave55 = Join-Path $tempDir "reference_limit_5p5.csv"
$null = Invoke-SimCase "wave-limit3p8" 36 1.8 10 2 3.8 $wave38
$null = Invoke-SimCase "wave-limit5p5" 36 1.8 10 2 5.5 $wave55

$rows | Export-Csv -LiteralPath $summaryPath -NoTypeInformation -Encoding utf8
Write-Host "Saved sweep: $summaryPath"

& python $plotScript `
    --summary $summaryPath `
    --wave-3p8 $wave38 `
    --wave-5p5 $wave55 `
    --out-dir (Join-Path $repoRoot "docs")
if ($LASTEXITCODE -ne 0) {
    throw "Plot generation failed."
}
