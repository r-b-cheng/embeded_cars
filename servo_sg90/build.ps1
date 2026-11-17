#requires -Version 7.0
param([string]$ToolchainBin, [switch]$LegacySweep)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $true

function Resolve-ArmTool([string]$Name) {
    if ($ToolchainBin) {
        return (Get-Item -LiteralPath (Join-Path $ToolchainBin "$Name.exe")).FullName
    }
    return (Get-Command -Name "$Name.exe" -CommandType Application | Select-Object -First 1).Source
}

$cc = Resolve-ArmTool 'arm-none-eabi-gcc'
$objcopy = Resolve-ArmTool 'arm-none-eabi-objcopy'
$size = Resolve-ArmTool 'arm-none-eabi-size'
$nm = if (-not $LegacySweep) { Resolve-ArmTool 'arm-none-eabi-nm' }
$source = if ($LegacySweep) { 'main.c' } else { 'hold.c' }
$name = if ($LegacySweep) { 'servo_sg90' } else { 'servo_sg90_hold' }

Push-Location -LiteralPath $PSScriptRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $flags = @(
        '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-std=c11',
        '-Os', '-g3', '-ffreestanding', '-fno-unwind-tables', '-fno-asynchronous-unwind-tables',
        '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra', '-Werror',
        '-nostdlib', '-Wl,--gc-sections,-z,noexecstack,--fatal-warnings', "-Wl,-Map=build/$name.map",
        '-T', 'tm4c123gh6pm.ld', 'startup.S', $source,
        '-o', "build/$name.elf"
    )
    & $cc @flags
    & $objcopy -O binary "build/$name.elf" "build/$name.bin"
    & $size "build/$name.elf"
    if (-not $LegacySweep) {
        $symbols = [ordered]@{}
        foreach ($line in (& $nm -n "build/$name.elf")) {
            if ($line -match '^\s*([0-9a-fA-F]+)\s+\S\s+(control_\w+)\s*$') {
                $symbols[$Matches[2]] = [Convert]::ToUInt32($Matches[1], 16)
            }
        }
        $required = @('control_target_us', 'control_enable', 'control_active', 'control_applied_us', 'control_millis')
        foreach ($key in $required) {
            if (-not $symbols.Contains($key) -or $symbols[$key] -lt 0x20000000 -or $symbols[$key] -ge 0x20008000) {
                throw "Invalid control mailbox symbol: $key"
            }
        }
        $metadata = [ordered]@{
            image_sha256 = (Get-FileHash "build/$name.bin" -Algorithm SHA256).Hash.ToLowerInvariant()
            symbols = $symbols
        }
        $metadata | ConvertTo-Json -Depth 4 | Set-Content "build/$name.symbols.json" -Encoding utf8NoBOM
    }
    Get-Item -LiteralPath "build/$name.bin" | Select-Object FullName,Length
} finally {
    Pop-Location
}
