#!/bin/bash
# Pair a PS4 controller. Hold Share + PS until the light bar double-flashes, then run this.
# No sudo needed (user is in the bluetooth group). Pairing has to happen inside one
# bluetoothctl session with a live agent, so all commands go through one pipe.
WAIT=${1:-90}
echo "Scanning up to ${WAIT}s for 'Wireless Controller'..."
{
	echo "agent NoInputNoOutput"; echo "default-agent"; echo "scan on"
	mac=""
	for i in $(seq $WAIT); do
		sleep 1
		mac=$(bluetoothctl devices | awk '/Wireless Controller/ {print $2; exit}')
		[ -n "$mac" ] && break
	done
	if [ -n "$mac" ]; then
		echo "pair $mac"; sleep 12
		echo "trust $mac"; echo "connect $mac"; sleep 8
	fi
	echo "scan off"; sleep 1; echo "quit"
} | bluetoothctl 2>&1 | sed "s/\x1b\[[0-9;]*m//g" | grep -a -E "Pairing|trust|Connection|Failed|not available"
bluetoothctl devices | grep "Wireless Controller"
ls -l /dev/input/js* 2>/dev/null || echo "No /dev/input/js* yet"
