#!/usr/bin/env python3
"""Read back the calibration stored on the OAK-D-Lite (depthai v2). Read-only."""
import depthai as dai

with dai.Device() as d:
    print("mxid", d.getMxId())
    try:
        c = d.readCalibration2(); e = c.getEepromData()
        print("user calib: version", e.version, "board", repr(e.boardName), "cams", len(e.cameraData))
        print("  stereo baseline cm:", round(c.getBaselineDistance(), 3))
        for sock, cd in e.cameraData.items():
            print("  ", sock, f"{cd.width}x{cd.height}", "hfov %.1f" % cd.specHfovDeg if hasattr(cd, "specHfovDeg") else "")
        print("  rgb lens position:", e.lensPosition if hasattr(e, "lensPosition") else "n/a")
    except Exception as ex:
        print("user calib ERR:", ex)
    try:
        f = d.readFactoryCalibration().getEepromData()
        print("factory calib: version", f.version, "cams", len(f.cameraData))
    except Exception as ex:
        print("factory calib ERR:", str(ex)[:80])
