# AGENTS.md

This agent file is the default for recreating the udev rule for the USB ACM device.

## Task: Write udev rule for /dev/ttyACM0 (CH340/CH9102)

- Target device: `/dev/ttyACM0`
- VID:PID: `1a86:55d3`
- Serial: `5AAF267162`
- Symlink: `/dev/ttyACM_mydevice`
- Permissions: `MODE="0660"`, `GROUP="dialout"`

### Rule contents

```
# Stable symlink for CH340/CH9102-based USB serial device
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5AAF267162", SYMLINK+="ttyACM_mydevice", GROUP="dialout", MODE="0660"
```

### Steps

1. Write the rule to `/etc/udev/rules.d/99-usb-serial.rules`.
2. Reload udev rules and trigger:
   - `udevadm control --reload-rules`
   - `udevadm trigger`
3. Verify the symlink exists: `ls -l /dev/ttyACM_mydevice`.

### Local copy

Also write the same contents to `/home/peter/99-usb-serial.rules` for easy reuse.
