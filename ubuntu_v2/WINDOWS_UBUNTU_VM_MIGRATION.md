# Windows Ubuntu VM migration

The current UTM virtual machine is ARM64. Do not copy its running `qcow2`
disk to a typical x86_64 Windows VM. Transfer the project bundle and the
prebuilt `linux/amd64` Docker image instead.

## Target VM requirements

- Ubuntu 22.04 or newer, x86_64/amd64
- Bridged networking so the VM and robot are on the same network
- Docker Engine and the Docker Compose plugin
- TCP port 9999 allowed from the Windows host

## Install with the prebuilt image

Copy these two files into the Ubuntu VM:

- `robot-control-v2-server-source.tar.gz`
- `robot-control-v2-gateway-amd64.tar`

Extract the source beside the image archive, then run:

```bash
tar -xzf robot-control-v2-server-source.tar.gz
cd ubuntu_v2
sudo ./scripts/install_prebuilt_amd64.sh ../robot-control-v2-gateway-amd64.tar
```

The installer loads the native amd64 image, installs the application in
`/opt/robot-control-v2`, preserves any existing operations database, and
enables the gateway at boot.

## Verify

```bash
docker compose -f /opt/robot-control-v2/compose.yaml ps
sudo ss -ltnp | grep :9999
ping -c 2 raspberrypi.local
```

Set the Windows GUI server address to the Ubuntu VM's bridged IPv4 address and
keep command port 9999. Copy the same command token from `.env` into the GUI.

## Operations history

The runtime database is `/opt/robot-control-v2/data/operations.sqlite3` in the
Ubuntu VM. Copy it only while the gateway is stopped:

```bash
sudo systemctl stop robot-control-v2.service
sudo cp operations.sqlite3 /opt/robot-control-v2/data/operations.sqlite3
sudo systemctl start robot-control-v2.service
```

The current VM database is not part of the source bundle unless it is exported
separately.
