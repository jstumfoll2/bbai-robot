# Board system files

Debian 10 Buster, kernel 4.14.108-ti-r136 (2020-04-06 TIDL image), ROS Melodic from source.

- `uEnv.txt`: `/boot/uEnv.txt`, which boots the Robotics Cape device tree.
- `blacklist-btsdio.conf`: `/etc/modprobe.d/`. Stops the bogus SDIO `hci0` so the UART6 Bluetooth is `hci0`.
- The Bluetooth boot service is `ros/bbai_teleop/bbai-bluetooth.service` (copy to `/etc/systemd/system/` and enable).
- `bin/ticklog.py`: logs eQEP encoder positions to `/tmp/ticks.log`, used for the encoder tests.
