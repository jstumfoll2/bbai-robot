#!/usr/bin/env python3
"""Headless OAK-D-Lite smoke test: device info, RGB preview + stereo depth FPS."""
import time, os, sys
import depthai as dai

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 15

p = dai.Pipeline()
cam = p.create(dai.node.ColorCamera)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setPreviewSize(300, 300)
cam.setInterleaved(False)
cam.setFps(15)
monoL = p.create(dai.node.MonoCamera); monoR = p.create(dai.node.MonoCamera)
for m, s in ((monoL, dai.CameraBoardSocket.LEFT), (monoR, dai.CameraBoardSocket.RIGHT)):
    m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    m.setBoardSocket(s); m.setFps(15)
st = p.create(dai.node.StereoDepth)
st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
monoL.out.link(st.left); monoR.out.link(st.right)
xr = p.create(dai.node.XLinkOut); xr.setStreamName("rgb"); cam.preview.link(xr.input)
xd = p.create(dai.node.XLinkOut); xd.setStreamName("depth"); st.depth.link(xd.input)

t0 = time.time()
with dai.Device(p) as dev:
    print("boot s:", round(time.time() - t0, 1))
    print("mxid:", dev.getMxId(), "usb:", dev.getUsbSpeed().name)
    print("cams:", [c.name for c in dev.getConnectedCameras()])
    try: print("eeprom:", dev.readCalibration().getEepromData().productName)
    except Exception as e: print("eeprom err", e)
    qr = dev.getOutputQueue("rgb", 4, False); qd = dev.getOutputQueue("depth", 4, False)
    n = {"rgb": 0, "depth": 0}; t1 = time.time(); last = None
    while time.time() - t1 < secs:
        if qr.tryGet() is not None: n["rgb"] += 1
        d = qd.tryGet()
        if d is not None: n["depth"] += 1; last = d
        time.sleep(0.005)
    dt = time.time() - t1
    print("fps rgb %.1f depth %.1f" % (n["rgb"] / dt, n["depth"] / dt))
    if last is not None:
        f = last.getFrame(); c = f[f.shape[0] // 2, f.shape[1] // 2]
        print("depth frame", f.shape, "center mm:", int(c))
    t = dev.getChipTemperature(); print("chip temp C: %.1f" % t.average)
    print("load", os.getloadavg())
