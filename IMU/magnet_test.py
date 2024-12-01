import py_lis3mdl
import time

sensor = py_lis3mdl.LIS3MDL()
sensor.mode_continuous()

while True:
    m = sensor.get_bearing()
    print(m)
