#!/usr/bin/env python3
"""Save RGB stills at several manual lens positions (plus autofocus) and count ChArUco markers in each.

Used to debug "color camera finds 0 markers" during calibration. Output: Camera/debug/*.jpg
Run with the depthai v2 env: C:/Users/jstum/venvs/oak-calib-v2/Scripts/python.exe rgb_focus_probe.py
"""
import os, time
import cv2
import depthai as dai

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug")
os.makedirs(OUT, exist_ok=True)
aruco = cv2.aruco
DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

p = dai.Pipeline()
cam = p.create(dai.node.ColorCamera)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
cam.setFps(5)
ctrl_in = p.create(dai.node.XLinkIn); ctrl_in.setStreamName("ctrl"); ctrl_in.out.link(cam.inputControl)
xo = p.create(dai.node.XLinkOut); xo.setStreamName("isp"); cam.isp.link(xo.input)

def markers(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, DICT)
    return 0 if ids is None else len(ids), float(gray.mean()), float(cv2.Laplacian(gray, cv2.CV_64F).var())

with dai.Device(p) as dev:
    q = dev.getOutputQueue("isp", 2, False); cq = dev.getInputQueue("ctrl")
    def grab(label, settle=2.5):
        t = time.time()
        while time.time() - t < settle: q.tryGet(); time.sleep(0.05)
        img = q.get().getCvFrame()
        n, mean, sharp = markers(img)
        small = cv2.resize(img, (img.shape[1] // 3, img.shape[0] // 3))
        cv2.imwrite(os.path.join(OUT, f"rgb_{label}.jpg"), small)
        print(f"{label:>10}: markers {n:3d}  brightness {mean:5.1f}  sharpness {sharp:8.1f}  size {img.shape[1]}x{img.shape[0]}", flush=True)
    c = dai.CameraControl(); c.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO); c.setAutoFocusTrigger(); cq.send(c)
    grab("autofocus", 5)
    for lp in (90, 110, 135, 150, 170, 190):
        c = dai.CameraControl(); c.setManualFocus(lp); cq.send(c)
        grab(f"lens{lp}")
print("saved to", OUT)
