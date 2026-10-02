#!/bin/sh
# Bring up the BeagleBone AI's onboard Bluetooth (Broadcom 43455 in the
# AzureWave AW-CM256SM module, HCI on UART6). Run as root; bbai-bluetooth.service
# runs it at boot. Needs a device tree with uart6 (serial@48068000) status = "okay".
set -e

# btsdio grabs a bogus hci0 on the SDIO bus; the real BT HCI is on the UART
modprobe -r btsdio 2>/dev/null || true

tty=""
for d in /sys/class/tty/ttyS*; do
	case "$(readlink -f $d/device)" in *48068000.serial) tty=/dev/$(basename $d) ;; esac
done
[ -n "$tty" ] || { echo "UART6 (48068000) has no tty: enable it in the device tree"; exit 1; }
echo "Bluetooth UART is $tty"

pkill -f "[h]ciattach $tty" || true
# BCM4345C0.hcd in /lib/firmware/brcm is used if present; the ROM firmware works without it
# A fresh chip talks at 115200; one that was already set up (service restart)
# is still at 3 Mbaud, so fall back to starting at that speed
hciattach "$tty" bcm43xx 3000000 flow || hciattach -s 3000000 "$tty" bcm43xx 3000000 flow
for i in 1 2 3 4 5 6 7 8 9 10; do hciconfig hci0 >/dev/null 2>&1 && break; sleep 1; done
hciconfig hci0 up

# Latency: no sniff mode on new links (gamepad reports stay at full rate), and
# no Wi-Fi power save (the 43455 shares its radio between Wi-Fi and BT)
hciconfig hci0 lp RSWITCH
iwconfig wlan0 power off 2>/dev/null || true
hciconfig hci0
