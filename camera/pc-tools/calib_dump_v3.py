#!/usr/bin/env python3
"""DepthAI v3 calibration dump (Luxonis example), also saving the output to a file for support."""
import json, sys, datetime
import depthai as dai

out = []
def log(s):
    print(s); out.append(s)

log(f"{datetime.datetime.now().isoformat()}  depthai {dai.__version__}")
device = dai.Device(dai.UsbSpeed.HIGH)
log(f"MxID: {device.getDeviceId() if hasattr(device, 'getDeviceId') else device.getMxId()}")
log(f"Is EEPROM available: {device.isEepromAvailable()}")
try:
    log(f"User calibration: {json.dumps(device.getCalibration().eepromToJson(), indent=2)}")
except Exception as ex:
    log(f"No user calibration: {ex}")
try:
    log(f"Factory calibration: {json.dumps(device.readFactoryCalibration().eepromToJson(), indent=2)}")
except Exception as ex:
    log(f"No factory calibration: {ex}")

path = sys.argv[1] if len(sys.argv) > 1 else "calib_dump_v3_output.txt"
with open(path, "w") as f:
    f.write("\n".join(out) + "\n")
print(f"saved {path}")
