#!/usr/bin/env bash
set -eo pipefail

# The bridged Ubuntu VM cannot be reached directly from this Mac because of
# the hypervisor's hairpin behavior. The Raspberry Pi can reach both peers, so
# keep a local GUI endpoint alive through the robot as an SSH jump.
exec /usr/bin/ssh \
    -N \
    -T \
    -o BatchMode=yes \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    -o ConnectTimeout=8 \
    -L 127.0.0.1:9999:172.30.1.81:9999 \
    pi@172.30.1.18
