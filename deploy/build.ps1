# Cross-compile both daemons for the Raspberry Pi Zero (ARMv6) into dist/.
# Usage: .\deploy\build.ps1   (from the madroomba-core directory)
$ErrorActionPreference = 'Stop'
$env:GOOS = 'linux'; $env:GOARCH = 'arm'; $env:GOARM = '6'
try {
    go build -ldflags '-s -w' -o dist/madroombad ./cmd/madroombad
    go build -ldflags '-s -w' -o dist/basestationd ./cmd/basestationd
    Write-Host 'OK ->' (Get-ChildItem dist | ForEach-Object Name)
    Write-Host 'Deploy: scp dist/madroombad pi@<robot>:/home/pi/ ; scp dist/basestationd pi@<base>:/home/pi/'
} finally {
    $env:GOOS = ''; $env:GOARCH = ''; $env:GOARM = ''
}
