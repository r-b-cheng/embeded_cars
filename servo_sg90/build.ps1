#requires -Version 7.0
param([string]$ToolchainBin)

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

Push-Location -LiteralPath $PSScriptRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $flags = @(
        '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-std=c11',
        '-Os', '-ffreestanding', '-fno-unwind-tables', '-fno-asynchronous-unwind-tables',
        '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra', '-Werror',
        '-nostdlib', '-Wl,--gc-sections,-z,noexecstack,--fatal-warnings', '-Wl,-Map=build/servo_sg90.map',
        '-T', 'tm4c123gh6pm.ld', 'startup.S', 'main.c',
        '-o', 'build/servo_sg90.elf'
    )
    & $cc @flags
    & $objcopy -O binary build/servo_sg90.elf build/servo_sg90.bin
    & $size build/servo_sg90.elf
    Get-Item -LiteralPath build/servo_sg90.bin | Select-Object FullName,Length
} finally {
    Pop-Location
}
