# Build the NavBot Console Windows installer.
#
#   powershell -ExecutionPolicy Bypass -File app\desktop\packaging\windows\build.ps1
#   -> app\desktop\dist\navbot-console-<version>-windows-setup.exe
#
# Needs Python 3.10+. If Inno Setup 6 is not installed, falls back to a
# portable zip (get Inno from https://jrsoftware.org/isdl.php).
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..\..")     # -> app\desktop

python -m pip install --quiet -r requirements.txt "pyinstaller>=6"
python -m PyInstaller --noconfirm packaging\navbot_console.spec
$ver = (python -c "import navbot_console; print(navbot_console.__version__)").Trim()

$iscc = @(
    "iscc.exe",
    "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
    "$env:ProgramFiles\Inno Setup 6\ISCC.exe"
) | Where-Object { Get-Command $_ -ErrorAction SilentlyContinue } | Select-Object -First 1

if ($iscc) {
    & $iscc "/DMyAppVersion=$ver" packaging\windows\installer.iss
    if ($LASTEXITCODE -ne 0) { throw "ISCC failed with exit code $LASTEXITCODE" }
    Write-Host "OK: app\desktop\dist\navbot-console-$ver-windows-setup.exe"
} else {
    $zip = "dist\navbot-console-$ver-windows-portable.zip"
    Compress-Archive -Path dist\navbot-console\* -DestinationPath $zip -Force
    Write-Host "Inno Setup not found - built portable zip instead: app\desktop\$zip"
}
