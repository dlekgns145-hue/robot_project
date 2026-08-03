$ErrorActionPreference = "Stop"

$GuiDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$VenvDir = Join-Path $GuiDir ".venv"

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -m venv $VenvDir
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python -m venv $VenvDir
} else {
    throw "Python 3.10 or newer is required. Install it from python.org first."
}

$Python = Join-Path $VenvDir "Scripts\python.exe"
& $Python -m pip install --upgrade pip
& $Python -m pip install -r (Join-Path $GuiDir "requirements.txt")

Write-Host "GUI installation complete. Double-click run_gui_windows.vbs."
