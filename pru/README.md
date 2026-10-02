# PRU firmware

The `gpio_rpmsg` firmware for PRU2_0 lives in the librobotcontrol fork, checked out here at
`../librobotcontrol` (branch `v1.1_AIfixes`), under `pru_firmware/customfw/`.

It exposes `/dev/rpmsg_pru32`, which librobotcontrol uses as "gpiochip 11". Motor 1's B direction
pin (P9.13) is only reachable through it, so `rc_motor_init` fails without it.

- Installed on the board as `/lib/firmware/am57xx-pru2_0-fw`.
- Not started at boot. `bbai_base`'s `base_node.py` starts remoteproc2 itself.
- Build: `cd librobotcontrol/pru_firmware/customfw && make` (on the board, with the TI PRU toolchain).
