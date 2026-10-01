#!/usr/bin/env python3
# Log eQEP encoder positions to /tmp/ticks.log whenever they change (2 h)
import glob, time
P=sorted(glob.glob("/sys/devices/platform/44000000.ocp/*.epwmss/*.eqep/position"))
last=None; end=time.time()+7200
with open("/tmp/ticks.log","w",buffering=1) as f:
    f.write("# %s\n"%P)
    while time.time()<end:
        v=[int(open(p).read()) for p in P]
        if v!=last: f.write("%.2f %s\n"%(time.time(),v)); last=v
        time.sleep(0.02)
