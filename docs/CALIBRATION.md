# Recalibrating the OAK-D-Lite (fallback if Luxonis has no file)

Do this on the Windows PC: the calibration tool needs a screen to show the live preview. Plug the camera into a USB3 port on the PC.

## 1. Print and measure the target

1. Print `charuco_11x8.pdf` (from `~/src/depthai`, or the depthai repo) at **100% / actual size**, with no "fit to page".
2. Glue or tape it **perfectly flat** onto something rigid, like foam board or a clipboard. Any bend ruins the calibration.
3. Measure with a ruler, in cm:
   - **square size**: the side of one black chessboard square
   - **marker size**: the side of the ArUco marker inside a white square, usually about 0.75 × the square size

   Write both numbers down.

## 2. Calibration tool (already set up on the PC)

- **Code:** `Camera\calibration\` is Luxonis's `depthai` repo, checked out on local branch `calib-v2` at commit `07a1b78c` (2025-11-05).
  - That's the last calibrate.py that runs on **DepthAI v2**: it picks v2 or v3 from the installed version.
- **Why not current `main` / DepthAI v3:** with this camera's empty EEPROM, the v3 firmware crashes as soon as the sensors start. Its crash report shows `PlgSrcMipi: Invalid config steps` and `RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE`.
  - v3 `main` also has a syntax typo at calibrate.py line 643.
- **Python environment:** `C:\Users\jstum\venvs\oak-calib-v2`, Python 3.11, depthai 2.33.0, OpenCV 4.5.5.62. It's kept outside OneDrive on purpose.
  - Rebuild with `py -3.11 -m venv ...`, then `pip install "depthai<3" "numpy>=1.26.4,<2" opencv-python==4.5.5.62 opencv-contrib-python==4.5.5.62 scipy matplotlib "pyqt5>5,<5.15.6" Qt.py packaging requests`.
- `C:\Users\jstum\venvs\oak-calib` (depthai v3) is still there for the Luxonis dump script only.

## 3. Run calibration

```powershell
cd "C:\Users\jstum\OneDrive\Robot 2.0\Camera\calibration"
C:\Users\jstum\venvs\oak-calib-v2\Scripts\python.exe calibrate.py -s 2.48 -ms 1.86 -brd OAK-D-LITE -ab 60 -dst C:\Users\jstum\oak-dataset
```

- It asks "This folders content will be deleted ... Proceed? (y/n)". Answer `y`.
- `-dst` keeps the captured images out of OneDrive, which locked the default `dataset` folder.
- Over USB2 the preview runs at about 3 fps, which is fine for calibration. A USB3 cable and port would be faster.

- `-s` and `-ms` are the measured square and marker sizes in **cm**. Our print is 24.8 mm squares, and the markers should be about 18.6 mm.
- `-ab 60` sets anti-banding for 60 Hz lighting.

1. A window opens showing the RGB, left and right previews. Press **s** to start.
2. The tool asks for about **13 poses**: board centred, tilted left, right, up and down, then closer and farther, in the corners, and so on. Match the outline it shows on screen. Hold the board still until it captures. The board must be fully visible in **all three** cameras.
3. Good lighting, no glare on the paper, and the board should fill a decent part of the frame.
4. After the last pose it computes the calibration and prints the **reprojection error**. Aim for **< 1.0**, ideally < 0.5. If it's worse, run it again.
5. When asked, let it **flash the result to the EEPROM**. This replaces the empty calibration.

## 4. Verify

Put the camera back on the BeagleBone and run `python3 ~/oak_calcheck.py`. The "user" line should now list 3 cameras.

Then run `python3 ~/dog_detect.py 30`. The "RGB camera calibration missing" and "ROI ... not a valid rectangle" errors should be gone. Point it at something a known distance away and check the `z` reading.

If Luxonis sends the factory file instead, skip all of this. Copy the .json to the board and I'll flash it.
