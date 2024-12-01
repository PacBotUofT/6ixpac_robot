import smbus
import time

# Define the I2C bus (0 for Pi 1, 1 for Pi 2 and newer)
I2C_BUS = 1

# I2C address of the GY-521 sensor
GY521_ADDRESS = 0x68

# Register addresses for sensor data
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

# Create an instance of the I2C bus
bus = smbus.SMBus(I2C_BUS)

# Initialize the GY-521 sensor
def setup_gy521():
    # Power management register (power on)
    bus.write_byte_data(GY521_ADDRESS, 0x6B, 0x00)

# Read sensor data
def read_data(register):
    high = bus.read_byte_data(GY521_ADDRESS, register)
    low = bus.read_byte_data(GY521_ADDRESS, register + 1)
    value = (high << 8) | low
    return value

if __name__ == "__main__":
    try:
        setup_gy521()
        while True:
            # Read accelerometer and gyroscope data
            accel_x = read_data(ACCEL_XOUT)
            accel_y = read_data(ACCEL_YOUT)
            accel_z = read_data(ACCEL_ZOUT)
            gyro_x = read_data(GYRO_XOUT)
            gyro_y = read_data(GYRO_YOUT)
            gyro_z = read_data(GYRO_ZOUT)

            # Print the sensor readings
            print(f"Accelerometer (X, Y, Z): ({accel_x}, {accel_y}, {accel_z})")
            print(f"Gyroscope (X, Y, Z): ({gyro_x}, {gyro_y}, {gyro_z})")
            print("")

            # Add a delay for readability
            time.sleep(1)

    except KeyboardInterrupt:
        print("Data reading stopped by user")
