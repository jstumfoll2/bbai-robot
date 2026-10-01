#!/usr/bin/env python3
"""Headless dog detector: MobileNet-SSD spatial detection on the OAK-D-Lite.

Prints one line per detection of interest: label, confidence, bearing (deg,
+ = right) and X/Z position in metres from the camera. Requires the camera's
EEPROM calibration (self-calibrated 2026-10-01).
"""
import math, sys, time
import depthai as dai

BLOB = "/home/debian/models/mobilenet-ssd_openvino_2021.4_6shave.blob"
LABELS = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
          "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
          "sheep", "sofa", "train", "tvmonitor"]
WATCH = {"dog", "cat", "person"}
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60

p = dai.Pipeline()
cam = p.create(dai.node.ColorCamera)
cam.setPreviewSize(300, 300)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(10)

left = p.create(dai.node.MonoCamera); right = p.create(dai.node.MonoCamera)
for m, s in ((left, dai.CameraBoardSocket.LEFT), (right, dai.CameraBoardSocket.RIGHT)):
    m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    m.setBoardSocket(s); m.setFps(10)
stereo = p.create(dai.node.StereoDepth)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # needs EEPROM calibration (flashed 2026-10-01)
stereo.setOutputSize(left.getResolutionWidth(), left.getResolutionHeight())
left.out.link(stereo.left); right.out.link(stereo.right)

nn = p.create(dai.node.MobileNetSpatialDetectionNetwork)
nn.setBlobPath(BLOB)
nn.setConfidenceThreshold(0.5)
nn.input.setBlocking(False)
nn.setBoundingBoxScaleFactor(0.5)
nn.setDepthLowerThreshold(100); nn.setDepthUpperThreshold(8000)
cam.preview.link(nn.input)
stereo.depth.link(nn.inputDepth)

xout = p.create(dai.node.XLinkOut); xout.setStreamName("det")
nn.out.link(xout.input)

HFOV = 69.0  # OAK-D-Lite RGB horizontal FOV, approx.
with dai.Device(p) as dev:
    q = dev.getOutputQueue("det", 4, False)
    t0 = time.time(); frames = 0; last_print = 0
    print("running %ds, usb %s" % (secs, dev.getUsbSpeed().name), flush=True)
    while time.time() - t0 < secs:
        msg = q.get(); frames += 1
        for d in msg.detections:
            lab = LABELS[d.label] if d.label < len(LABELS) else str(d.label)
            if lab not in WATCH:
                continue
            cx = (d.xmin + d.xmax) / 2
            bearing = (cx - 0.5) * HFOV
            x, z = d.spatialCoordinates.x / 1000, d.spatialCoordinates.z / 1000
            print("%6.1fs %-7s %3.0f%% bearing %+5.1f deg  x %+.2f m  z %.2f m" %
                  (time.time() - t0, lab, d.confidence * 100, bearing, x, z), flush=True)
        if time.time() - last_print > 10:
            print("-- %.1f det-fps" % (frames / (time.time() - t0)), flush=True)
            last_print = time.time()
print("done", flush=True)
