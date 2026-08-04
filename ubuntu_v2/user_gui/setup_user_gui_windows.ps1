$ErrorActionPreference = "Stop"

$UserGuiDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$UserGuiVenv = Join-Path $UserGuiDir ".venv"

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -m venv $UserGuiVenv
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python -m venv $UserGuiVenv
} else {
    throw "Python 3.10 or newer is required. Install it from python.org first."
}

if ($LASTEXITCODE -ne 0) {
    throw "Failed to create the Python virtual environment."
}

$UserGuiPython = Join-Path $UserGuiVenv "Scripts\python.exe"
& $UserGuiPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) {
    throw "Failed to upgrade pip."
}

& $UserGuiPython -m pip install -r (Join-Path $UserGuiDir "requirements.txt")
if ($LASTEXITCODE -ne 0) {
    throw "Failed to install GUI packages."
}

Write-Host "Robot Companion installation complete."
Write-Host "Next time, double-click run_user_gui_windows.vbs."
