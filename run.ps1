<#!
PowerShell equivalent of run.sh

Usage examples:
  # Using GDB/OpenOCD (default when USE_DFU not set)
  pwsh -File .\run.ps1 target\thumbv7em-none-eabihf\release\firefly-firmware

  # Using DFU mode
  $Env:USE_DFU=1; pwsh -File .\run.ps1 target\thumbv7em-none-eabihf\release\firefly-firmware

Assumptions:
  - arm-none-eabi-gdb, llvm-objcopy, dfu-util are on PATH
  - openocd is launched separately if not using DFU
  - First positional argument is the ELF file path produced by cargo
!#>

param(
	[Parameter(Position=0,Mandatory=$true)]
	[string]$ElfPath
)

if (-not (Test-Path $ElfPath)) {
	Write-Error "ELF file '$ElfPath' not found."; exit 1
}

$useDfu = [string]::IsNullOrWhiteSpace($Env:USE_DFU) -ne $true

if (-not $useDfu) {
	# GDB/OpenOCD path
	$openOcdRunning = Get-Process -Name openocd -ErrorAction SilentlyContinue
	if (-not $openOcdRunning) {
		Write-Host "openOCD is not running, please run:"
		Write-Host "  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg"
		exit 1
	}

	$gdbArgs = @(
		'-ex','target extended-remote localhost:3333'
		'-ex',"load $ElfPath"
		'-ex','set confirm off'
		'-ex',"file $ElfPath"
		'-ex','monitor arm semihosting enable'
	)

	Write-Host "Launching arm-none-eabi-gdb..."
	& arm-none-eabi-gdb @gdbArgs
	exit $LASTEXITCODE
} else {
	Write-Host "Using DFU mode to flash the firmware." -ForegroundColor Cyan
	$binOut = 'firmware.bin'
	Write-Host "Generating binary: $binOut"
	& llvm-objcopy -O binary $ElfPath $binOut
	if ($LASTEXITCODE -ne 0) { Write-Error "llvm-objcopy failed."; exit 1 }

	Write-Host "Flashing via dfu-util..."
	& dfu-util -a 0 --dfuse-address 0x08000000 -D $binOut
	exit $LASTEXITCODE
}
