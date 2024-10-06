import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# GPIO pin numbers for each sensor's XSHUT pin
XSHUT_PINS = [6, 13, 19, 26]

# New I2C addresses for each sensor
I2C_ADDRESSES = [0x29, 0x30, 0x31, 0x32]

# Setup GPIO
GPIO.setmode(GPIO.BCM)

# Set XSHUT pins to output and keep them high (enabled)
for pin in XSHUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize each VL53L0X sensor with a unique I2C address
sensors = []

def initialize_sensor(sensor_number):
    """Initializes a sensor, changes its I2C address, and adds it to the list."""
    # Bring this sensor out of shutdown (by pulling XSHUT low then high)
    GPIO.output(XSHUT_PINS[sensor_number], GPIO.LOW)
    time.sleep(0.01)  # Ensure the sensor has reset
    GPIO.output(XSHUT_PINS[sensor_number], GPIO.HIGH)
    time.sleep(0.01)  # Allow time for the sensor to boot

    # Initialize the sensor with the default address
    sensor = adafruit_vl53l0x.VL53L0X(i2c)

    # Change the I2C address for this sensor
    sensor.set_address(I2C_ADDRESSES[sensor_number])

    # Add the sensor object to the list
    sensors.append(sensor)

# Initialize each sensor one by one, setting a new I2C address
for i in range(4):
    initialize_sensor(i)

# Now, the sensors have unique addresses and can be used
for i, sensor in enumerate(sensors):
    distance = sensor.range
    print(f"Sensor {i + 1} distance: {distance} mm")

# Cleanup GPIO after use
GPIO.cleanup()
