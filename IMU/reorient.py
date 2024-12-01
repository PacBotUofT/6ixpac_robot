import py_lis3mdl
#from motors import motor_control
import time

sensor = py_lis3mdl.LIS3MDL()
sensor.mode_continuous()

def set_orientation():
    input("Press Enter to set target orientation")
    target = sensor.get_bearing_raw()
    return target

#+ is CW, - is CCW
def rotate_robot(direction):
    if direction > 0:
        #motorN.forward(0.5)
        #motorE.forward(0.5)
        #motorS.forward(0.5)
        #motorW.forward(0.5)
        print("Rotating CCW")
    else:
        #motorN.backward(0.5)
        #motorE.backward(0.5)
        #motorS.backward(0.5)
        #motorW.backward(0.5)
        print("Rotating CW")

if __name__ == "__main__":
    target = set_orientation()
    while True:
        try:
            diff = target - sensor.get_bearing_raw()
            if abs(diff) > (0.05 * target):
                rotate_robot(diff)
                diff = target - sensor.get_bearing_raw()
            else:
                print("Target orientation reached")
                time.sleep(0.5)
        except OSError as e:
            print(f"Error reading sensor data: {e}")
            # Handle the error, e.g., retrying, logging, etc.


