#!/usr/bin/env python3
"""Read-only: report user and factory calibration on the OAK device."""
import json, depthai as dai
with dai.Device() as dev:
    print("bootloader/usb:", dev.getUsbSpeed().name)
    for name, fn in (("user", dev.readCalibration2), ("factory", dev.readFactoryCalibration)):
        try:
            c = fn(); e = c.getEepromData()
            print(name, "board:", e.boardName, "product:", e.productName, "ver:", e.version,
                  "cams:", len(e.cameraData))
            if name == "factory" and len(e.cameraData):
                c.eepromToJsonFile("/home/debian/oak_factory_calib.json"); print("saved factory json")
        except Exception as ex:
            print(name, "ERR:", ex)
print("done")
