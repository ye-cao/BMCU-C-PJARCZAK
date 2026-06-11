$ErrorActionPreference = "Stop"

$OUT_DIR = "firmwares"
$PIO_ENV = "fw"

$txtMode = "which_to_choose_mode.txt"
$txtAutoload = "which_to_choose_autoload.txt"
$txtRgb = "which_to_choose_filament_rgb.txt"
$txtSlots = "which_to_choose_slots.txt"
$outGuide = "which_to_choose.txt"

if (!(Test-Path $txtMode))     { throw "Missing $txtMode" }
if (!(Test-Path $txtAutoload)) { throw "Missing $txtAutoload" }
if (!(Test-Path $txtRgb))      { throw "Missing $txtRgb" }
if (!(Test-Path $txtSlots))    { throw "Missing $txtSlots" }

$MODE_A1 = "standard(A1)"
$MODE_P1S = "high_force_load(P1S)"
$SOLO_RETRACT = "0.095f"
$RETRACTS = @("0.10","0.20","0.25","0.30","0.35","0.40","0.45","0.50","0.55","0.60","0.65","0.70","0.75","0.80","0.85","0.90")

if (Test-Path $OUT_DIR) { Remove-Item -Recurse -Force $OUT_DIR }
New-Item -ItemType Directory -Path $OUT_DIR | Out-Null
Copy-Item -Force $txtMode "$OUT_DIR/$outGuide"

$total = 0
$builds = @()

foreach ($p1s in @(0, 1)) {
    $modeDir = if ($p1s -eq 1) { $MODE_P1S } else { $MODE_A1 }
    $modeBase = "$OUT_DIR/$modeDir"
    New-Item -ItemType Directory -Path $modeBase -Force | Out-Null
    Copy-Item -Force $txtAutoload "$modeBase/$outGuide"

    foreach ($dm in @(1, 0)) {
        $dmDir = if ($dm -eq 1) { "AUTOLOAD" } else { "NO_AUTOLOAD" }
        $dmBase = "$modeBase/$dmDir"
        New-Item -ItemType Directory -Path $dmBase -Force | Out-Null
        Copy-Item -Force $txtRgb "$dmBase/$outGuide"

        foreach ($rgb in @(1, 0)) {
            $rgbDir = if ($rgb -eq 1) { "FILAMENT_RGB_ON" } else { "FILAMENT_RGB_OFF" }
            $base = "$dmBase/$rgbDir"

            foreach ($sub in @("SOLO","AMS_A","AMS_B","AMS_C","AMS_D")) {
                New-Item -ItemType Directory -Path "$base/$sub" -Force | Out-Null
            }
            Copy-Item -Force $txtSlots "$base/$outGuide"

            $total++
            $builds += @{
                OutPath = "$base/SOLO/solo_${SOLO_RETRACT}.bin"
                AmsNum = 0; Retract = "${SOLO_RETRACT}"; Dm = $dm; Rgb = $rgb; P1s = $p1s
            }

            foreach ($slot in @("A","B","C","D")) {
                $amsNum = switch ($slot) { "A"{0} "B"{1} "C"{2} "D"{3} }
                foreach ($r in $RETRACTS) {
                    $total++
                    $builds += @{
                        OutPath = "$base/AMS_$slot/ams_$($slot.ToLower())_${r}f.bin"
                        AmsNum = $amsNum; Retract = "${r}f"; Dm = $dm; Rgb = $rgb; P1s = $p1s
                    }
                }
            }
        }
    }
}

Write-Host "Total builds: $total"
Write-Host ""

$idx = 0
foreach ($b in $builds) {
    $idx++
    if (Test-Path $b.OutPath) {
        Write-Host "[$idx/$total] SKIP (exists) -> $($b.OutPath)"
        continue
    }
    Write-Host "[$idx/$total] P1S=$($b.P1s) DM=$($b.Dm) RGB=$($b.Rgb) AMS=$($b.AmsNum) RETRACT=$($b.Retract) -> $($b.OutPath)"

    $env:BAMBU_BUS_AMS_NUM = $b.AmsNum.ToString()
    $env:AMS_RETRACT_LEN = $b.Retract
    $env:BMCU_DM_TWO_MICROSWITCH = $b.Dm.ToString()
    $env:BMCU_ONLINE_LED_FILAMENT_RGB = $b.Rgb.ToString()
    $env:DBMCU_P1S = $b.P1s.ToString()
    $env:BMCU_SOFT_LOAD = "0"

    $env:PYTHONIOENCODING = "utf-8"
    $ErrorActionPreference = "Continue"
    $buildOutput = & pio run -e $PIO_ENV 2>&1
    $ErrorActionPreference = "Stop"

    $src = ".pio/build/$PIO_ENV/firmware.bin"
    if (!(Test-Path $src)) {
        Write-Host "FAILED!" -ForegroundColor Red
        $buildOutput | Select-String -Pattern "error|FAILED" | ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
        throw "Build failed at [$idx/$total]: $src not found"
    }

    $dir = Split-Path $b.OutPath -Parent
    if (!(Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    Copy-Item -Force $src $b.OutPath
}

Write-Host ""
Write-Host "DONE. Output: $OUT_DIR/"
