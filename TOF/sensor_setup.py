import time
import os
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# GPIO pin numbers for each sensor's XSHUT pin
XSHUT_PINS = [6, 13, 19, 26]

# New I2C addresses for each sensor
I2C_ADDRESSES = [0x30, 0x31, 0x32, 0x33]  # Assign non-conflicting addresses

# Setup GPIO
GPIO.setmode(GPIO.BCM)

# Set XSHUT pins to output and keep all sensors disabled (LOW)
for pin in XSHUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Disable all sensors initially

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize each VL53L0X sensor with a unique I2C address
sensors = []

def initialize_sensor(sensor_number):
    """Initializes a sensor, changes its I2C address, and adds it to the list."""
    print(f"Initializing sensor {sensor_number + 1}")

    # Ensure all sensors are disabled except the one being configured
    for i, pin in enumerate(XSHUT_PINS):
        if i == sensor_number:
            GPIO.output(pin, GPIO.HIGH)  # Enable the sensor to be configured

    time.sleep(0.1)  # Give the sensor time to boot

    try:
        # Initialize the sensor at the default I2C address (0x29)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)

        # Change the I2C address to the new one
        sensor.set_address(I2C_ADDRESSES[sensor_number])
        print(f"Sensor {sensor_number + 1} set to address {hex(I2C_ADDRESSES[sensor_number])}")

        # Add the sensor object to the list
        sensors.append(sensor)

    except Exception as e:
        print(f"Error initializing sensor {sensor_number + 1}: {e}")
        # Power down the sensor if there is an error
        GPIO.output(XSHUT_PINS[sensor_number], GPIO.LOW)
        time.sleep(0.1)

# Initialize each sensor one by one, setting a new I2C address
for i in range(4):
    initialize_sensor(i)

# Once all sensors are initialized, power them on and read distances
for i, sensor in enumerate(sensors):
    GPIO.output(XSHUT_PINS[i], GPIO.HIGH)  # Power on each sensor
    time.sleep(0.1)  # Give time for the sensor to boot

    try:
        distance = sensor.range
        print(f"Sensor {i + 1} distance: {distance} mm")
    except Exception as e:
        print(f"Error reading sensor {i + 1}: {e}")

# Cleanup GPIO after use
GPIO.cleanup()

