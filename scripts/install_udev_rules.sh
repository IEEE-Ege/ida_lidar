#!/usr/bin/env bash
# Installs the RPLIDAR C1 udev rule so it's reachable at the stable path
# /dev/lidar instead of the enumeration-order-dependent /dev/ttyUSBx.
set -euo pipefail

RULE_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/udev/99-rplidar-c1.rules"
RULE_DST="/etc/udev/rules.d/99-rplidar-c1.rules"

sudo cp "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

if ! groups "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to the dialout group — log out/in (or run 'newgrp dialout') for this to take effect."
fi

echo "Installed $RULE_DST"
echo "Plug in the RPLIDAR C1 and check: ls -la /dev/lidar"
