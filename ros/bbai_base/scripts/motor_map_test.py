#!/usr/bin/env python3
"""Spin each cape motor channel briefly and report which encoder moved.

Run with the wheels off the ground and bbai_base stopped. Each motor runs
forward at a low duty for a short burst; the encoder that changes the most
identifies the wheel, and its sign says whether +duty drives it forward.
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from base_node import start_motor_dir_pru  # noqa: E402

start_motor_dir_pru()

import rcpy  # noqa: E402
import rcpy.encoder as encoder  # noqa: E402
import rcpy.motor as motor  # noqa: E402

DUTY = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
BURST = 1.5
# Encoder map from the spin tests, and the sign that makes forward positive
WHEELS = {1: "left back", 2: "right back", 3: "left front", 4: "right front"}
FORWARD_SIGN = {1: 1, 2: -1, 3: 1, 4: -1}

try:
    for ch in (1, 2, 3, 4):
        for d in (DUTY, -DUTY):
            before = [encoder.get(e) for e in (1, 2, 3, 4)]
            motor.set(ch, d)
            time.sleep(BURST)
            motor.set(ch, 0.0)
            time.sleep(0.7)
            delta = [encoder.get(e) - b for e, b in zip((1, 2, 3, 4), before)]
            e = max(range(4), key=lambda i: abs(delta[i])) + 1
            fwd = delta[e - 1] * FORWARD_SIGN[e]
            print("motor %d duty %+.2f: encoder deltas %s -> %s, %s"
                  % (ch, d, delta, WHEELS[e] if delta[e - 1] else "nothing moved",
                     "forward" if fwd > 0 else "backward" if fwd < 0 else "-"))
            sys.stdout.flush()
finally:
    for ch in (1, 2, 3, 4):
        motor.set(ch, 0.0)
    rcpy.exit()
