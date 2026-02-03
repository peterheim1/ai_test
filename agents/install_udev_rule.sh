#!/usr/bin/env bash
set -euo pipefail

RULE_PATH=/etc/udev/rules.d/99-usb-serial.rules
LOCAL_COPY=/home/peter/99-usb-serial.rules

# Write rule to system rules directory (requires sudo).
sudo tee "$RULE_PATH" >/dev/null <<'RULE'
# Stable symlink for CH340/CH9102-based USB serial device
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5AAF267162", SYMLINK+="ttyACM_mydevice", GROUP="dialout", MODE="0660"
RULE

# Write local copy for reuse.
cat <<'RULE' > "$LOCAL_COPY"
# Stable symlink for CH340/CH9102-based USB serial device
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5AAF267162", SYMLINK+="ttyACM_mydevice", GROUP="dialout", MODE="0660"
RULE

# Reload and trigger udev.
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed $RULE_PATH and wrote local copy to $LOCAL_COPY"
echo "Verify with: ls -l /dev/ttyACM_mydevice"
