# Raspberry Pi setup

Both Pis are Raspberry Pi Zero W (ARMv6) running Raspberry Pi OS Bullseye. The
binaries are cross-compiled on a workstation (`deploy/build.ps1`) and copied
over SSH; nothing is built on the Pis.

## SSH access

Use a dedicated key and avoid passwords:

```powershell
ssh-keygen -t ed25519 -f $env:USERPROFILE\.ssh\id_ed25519_madroomba
scp $env:USERPROFILE\.ssh\id_ed25519_madroomba.pub pi@<host>:/tmp/k.pub
ssh pi@<host> "mkdir -p ~/.ssh && chmod 700 ~/.ssh && tr -d '\r' < /tmp/k.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && rm /tmp/k.pub"
```

Pitfalls seen on Windows: in PowerShell, `-N '""'` sets a passphrase made of
two quote characters (use `-N ''` and check with `ssh-keygen -y -P '' -f <key>`),
and piping a file through `type` can leave carriage returns in
`authorized_keys`, hence the `tr -d '\r'`.

Without `-i`, ssh only tries the default key names (`id_rsa`,
`id_ed25519`...). Map each Pi to its key in `~/.ssh/config`, so that plain
`ssh madroomba` or `ssh madserver` works:

```
Host madroomba madroomba.local
    User pi
    IdentityFile ~/.ssh/id_ed25519_madroomba
    IdentitiesOnly yes

Host madserver madserver.local
    User pi
    IdentityFile ~/.ssh/id_ed25519_madserver
    IdentitiesOnly yes
```

WSL has its own `~/.ssh`, separate from the Windows one. ssh refuses a
private key readable by others ("UNPROTECTED PRIVATE KEY FILE"): run
`chmod 600` on the private keys and the config, `chmod 644` on the `.pub`.

### Key login only

Once key login works, turn off password login on each Pi, one at a time.
Keep an SSH session open during the change, to undo it if needed:

```sh
sudo tee /etc/ssh/sshd_config.d/10-keys-only.conf <<'EOF'
# Key login only: no passwords, no root
PasswordAuthentication no
KbdInteractiveAuthentication no
PermitRootLogin no
EOF
sudo sshd -t && sudo sshd -T | grep -Ei 'passwordauth|kbdinteractive|permitroot'
sudo systemctl reload ssh
```

The `10-` prefix matters: sshd keeps the first value it reads, and some
images ship a `50-cloud-init.conf` that sets `PasswordAuthentication yes`.
From another terminal,
`ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password pi@<host>`
must now fail with `Permission denied (publickey)`, and `ssh <host>` must
still log in. To undo: remove the file and reload ssh. The `pi` password is
still used by `sudo` and the local console.

## Robot Pi: serial port

The Roomba is on the hardware UART, which must be free of the login console and
Bluetooth:

1. In `/boot/cmdline.txt`, remove `console=serial0,115200`.
2. In `/boot/config.txt`, add `enable_uart=1` and `dtoverlay=disable-bt`.
3. `sudo systemctl disable hciuart`, then reboot.

`/dev/serial0` should now point to `ttyAMA0`. The service user needs the
`dialout` and `gpio` groups (the unit sets them).

Back up the files before editing (`sudo cp /boot/config.txt /boot/config.txt.bak`).

## Camera (both Pis)

The cameras on this project are only detected by the legacy camera stack.
Symptoms of the wrong stack: `vcgencmd get_camera` shows `detected=0`,
`libcamera-vid --list-cameras` says "No cameras available", and the web page
shows no image.

In `/boot/config.txt`:

```
# camera_auto_detect=1          (commented out)
# dtoverlay=vc4-kms-v3d         (commented out, not compatible with the legacy stack)
start_x=1
gpu_mem=128
```

Reboot, then `vcgencmd get_camera` should print `supported=1 detected=1`.

If it still says `detected=0`, the camera is not seen electrically: check the
ribbon cable (the Pi Zero has a narrow connector and needs the matching cable),
the contact side, and that both latches are closed. Connect with the Pi powered
off.

The daemons try `rpicam-vid`, then `libcamera-vid`, then `raspivid`, and keep the
first that produces a frame within 8 s. On the legacy stack the first two
exit immediately and `raspivid` is used (the log shows
`libcamera-vid produced no frames, trying next`).

## systemd units

`deploy/madroombad.service` and `deploy/basestationd.service`. They run as
user `pi`, restart on failure after 5 s and start at boot. The robot unit
needs the serial and GPIO groups, the base unit needs `video`. Flags such as
`-base-url`, `-robot-url`, `-lidar-port` are set on the `ExecStart` line; run
`sudo systemctl daemon-reload` after editing the unit on the Pi.

Useful commands:

```sh
systemctl status madroombad            # or basestationd
journalctl -u madroombad -n 50 --no-pager
sudo systemctl restart madroombad
```

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Robot service restarts every 5 s, log says `open /dev/serial0: no such file` | UART not enabled, see "Robot Pi: serial port" |
| Log says `lidar: ... (retrying every 5s)` | No USB serial adapter found, or it cannot be opened. The page, teleoperation and camera still work; the map stays empty. The lidar is picked up as soon as it is plugged in. Set `-lidar-port` if `auto` picks the wrong device |
| Log says `lidar started` but the page shows 0 RPM | The port is open but no frame arrives. Check the lidar cable and that the adapter shares its ground with the Pi |
| Camera tile empty, log says `no usable camera command found` | Camera not detected, see "Camera" |
| Dock page shows motion right after a restart | Normal for a few seconds while the exposure settles; analysis is paused for 6 s after each camera start |
| "Robot hors ligne" on the base page | The robot daemon is down or `-robot-url` is wrong. Check `curl http://<base>:8081/proxy/robot/api/config` |
| Thin white line on the right edge of camera images | The camera stream has an almost white last pixel column. The page hides it by scaling images 1.2%; downloaded snapshots and the raw `/camera` stream still contain it |
| Dock state flips between docked and empty | Teach both references again with the lighting you normally have, or the camera moved |
