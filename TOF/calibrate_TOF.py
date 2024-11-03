from sensor_setup import sensors
import time

def log_calibration_data(sensor, known_distance, sensor_name="sensor_1", filename="sensor_calibration_log.txt"):
    """
    Logs calibration data for a sensor measuring a known fixed distance.
    
    Parameters:
    - sensor: The VL53L0X sensor object to calibrate.
    - known_distance: The actual known distance from the sensor to the target (in mm).
    - sensor_name: Identifier for the sensor being measured.
    - filename: The name of the file where data will be logged.
    """
    count = 0  
    max_count = 60  #TBD
    valid_measurements = [] #store valid measurements to track avg
    
    with open(filename, "w") as file:
        file.write("sensor_measured,actual_distance,measured_distance,offset\n")
        
        while count < max_count:
            try:
                readings = []
                for _ in range(10):
                    readings.append(sensor.range)
                    time.sleep(0.1)

                measured_distance = sum(readings) / len(readings)
                
                # Calculate offset
                offset = known_distance - measured_distance

                # Check for outlier using the is_outlier function
                if is_outlier(measured_distance, valid_measurements):
                    response = input(f"Outlier detected: Measured = {measured_distance} mm, Offset = {offset} mm. Continue? (y/n): ")
                    if response.lower() != 'y':
                        print("Measurement skipped.")
                        continue  # Skip this measurement
                
                # Write data to file
                file.write(f"{sensor_name},{known_distance},{measured_distance},{offset}\n")
                print(f"{sensor_name}: Actual = {known_distance} mm, Measured = {measured_distance} mm, Offset = {offset} mm")
                
                #add measurement to valid measurements after confirmation
                valid_measurements.append(measured_distance)

                count += 1

            except Exception as e:
                print(f"Error reading from {sensor_name}: {e}. Retrying...")
                time.sleep(0.5)  # Wait and retry on error

    print(f"Calibration complete. Data logged to {filename}")

def is_outlier(measured_distance, valid_measurements, threshold=0.2):
    if not valid_measurements:
        return False  # No measurements yet, so it can't be an outlier

    avg_measurement = sum(valid_measurements) / len(valid_measurements)
    return abs(measured_distance - avg_measurement) > threshold * avg_measurement