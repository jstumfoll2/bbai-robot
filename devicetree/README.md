# Device tree

The board boots `am5729-beagleboneai-roboticscape.dtb` (see `system/uEnv.txt`), built from
[beagleboard/BeagleBoard-DeviceTrees](https://github.com/beagleboard/BeagleBoard-DeviceTrees) at
`4a9c0a6` plus `roboticscape-bbai.patch`.

Changes on top of upstream (the 2022 work plus 2026-10-01):
- HDMI disabled; I2C4 enabled on P9.19/P9.20 for the cape.
- P9.13 (motor 1 direction B) muxed to `pr2_pru0_gpo15`, driven by the PRU `gpio_rpmsg` firmware (see `pru/`).
- `PIN_INPUT_PULLUP` on all eight eQEP encoder pins. The A1230 Hall sensors are open-drain, so the
  encoders count nothing without the pull-ups.
- `&uart6` enabled for the Bluetooth HCI (BCM43455). It is `/dev/ttyS5` on the board.

The full `.dts` files are copied here as well, so the patch is only needed to rebase onto a newer upstream.

Build and install (needs sudo, then a reboot):

```bash
cd ~/src/BeagleBoard-DeviceTrees
git apply /path/to/bbai-robot/devicetree/roboticscape-bbai.patch
./udt.sh
```
