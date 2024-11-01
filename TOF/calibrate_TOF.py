from sensor_setup import sensors

def calibrate_sensor(sensor, known_distance):
    """Calibrates a VL53L0X sensor by measuring a known distance."""
    # Measure distance multiple times and calculate the average
    readings = [sensor.range for _ in range(10)]
    measured_distance = sum(readings) / len(readings)
    
    # Calculate offset (difference from known distance)
    offset = known_distance - measured_distance
    
    print(f"Calibration offset for sensor: {offset} mm")
    return offset

# Example usage
known_distance = 100  # mm
for sensor in sensors:
    offset = calibrate_sensor(sensor, known_distance)
    # Apply offset correction to future readings for this sensor
    sensor.offset = offset

# Use the offset during distance measurement
for i, sensor in enumerate(sensors):
    distance = sensor.range + sensor.offset
    print(f"Calibrated distance for sensor {i + 1}: {distance} mm")