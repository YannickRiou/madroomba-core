# Build and deploy the daemons to the Pis.
#   .\deploy\deploy.ps1                  # robot and base
#   .\deploy\deploy.ps1 -Only base       # or: -Only robot
#   .\deploy\deploy.ps1 -RobotKey C:\path\to\another_key
# Run from the madroomba-core directory. Needs ssh/scp and passwordless sudo on the Pis.
# Keys: a key given with -RobotKey / -BaseKey is the only one tried. Without it, the
# per-Pi key of docs/pi-setup.md (~/.ssh/id_ed25519_madroomba, ~/.ssh/id_ed25519_madserver)
# is tried first when it exists, then ssh's default identities.
param(
    [ValidateSet('both', 'robot', 'base')][string]$Only = 'both',
    [string]$Robot = 'pi@madroomba.local',
    [string]$Base = 'pi@madserver',
    [string]$RobotKey = '',
    [string]$BaseKey = ''
)
$ErrorActionPreference = 'Stop'

function Deploy([string]$target, [string]$name, [string]$key, [string]$defaultKey) {
    $sshOpts = @('-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10')
    if ($key) { $sshOpts += @('-i', $key, '-o', 'IdentitiesOnly=yes') }
    elseif (Test-Path $defaultKey) {
        # with -i, ssh stops trying its default keys: list them after the per-Pi key
        $sshOpts += @('-i', $defaultKey)
        foreach ($k in 'id_ed25519', 'id_ecdsa', 'id_rsa') {
            if (Test-Path "$HOME\.ssh\$k") { $sshOpts += @('-i', "$HOME\.ssh\$k") }
        }
    }
    Write-Host "=== $name -> $target"
    scp @sshOpts "dist/$name" "deploy/$name.service" "${target}:/tmp/"
    if ($LASTEXITCODE) { throw "scp to $target failed" }
    # stop first so the binary is not busy, then install, reload units and start
    $remote = "sudo systemctl stop $name; install -m755 /tmp/$name /home/pi/$name && " +
        "sudo mv /tmp/$name.service /etc/systemd/system/ && sudo systemctl daemon-reload && " +
        "sudo systemctl enable $name >/dev/null 2>&1; sudo systemctl start $name; sleep 6; " +
        "systemctl is-active $name; journalctl -u $name -n 8 --no-pager"
    ssh @sshOpts $target $remote | Out-Host
    if ($LASTEXITCODE) { throw "ssh to $target failed" }
}

& "$PSScriptRoot\build.ps1"
if ($Only -ne 'base')  { Deploy $Robot 'madroombad' $RobotKey "$HOME\.ssh\id_ed25519_madroomba" }
if ($Only -ne 'robot') { Deploy $Base 'basestationd' $BaseKey "$HOME\.ssh\id_ed25519_madserver" }
