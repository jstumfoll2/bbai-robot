# OAK-D-Lite

## board/
Run on the BeagleBone AI (depthai 2.19.1, Python 3.7).
- `oak_test.py`: streams RGB and depth.
- `oak_calcheck.py`: reads the calibration (read-only).
- `dog_detect.py`: MobileNet-SSD dog detection on the camera's Myriad X. Needs
  `~/models/mobilenet-ssd_openvino_2021.4_6shave.blob` (not in git; from the Luxonis model zoo).
The chase controller is the `ros/bbai_chase` package.

## pc-tools/
Run on the Windows PC with the camera plugged in. See `docs/CALIBRATION.md`.
- `calib_dump_v3.py`, `calib_dump_v3_output.txt`: the depthai v3 dump Luxonis asked for.
- `calib_readback.py`: reads back the stored calibration (depthai v2).
- `rgb_focus_probe.py`: captures RGB frames at several lens positions.
- `oak-d-lite_19443010213DF71200_calib.json`: backup of the self-calibration flashed on 2026-10-01
  (epipolar error 0.36, baseline 7.43 cm, RGB lens 72). Flash it back with `flashCalibration2()`.

Calibration tool: [luxonis/depthai](https://github.com/luxonis/depthai) at `07a1b78c` (the last
`calibrate.py` that runs on depthai v2), with its `depthai_calibration` submodule moved to `bedcf34`
(luxonis/depthai-calibration PR #27, the low-count fix). Reproduce with:

```bash
git clone https://github.com/luxonis/depthai.git && cd depthai
git checkout 07a1b78c && git submodule update --init
cd depthai_calibration && git checkout bedcf34
```
