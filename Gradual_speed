#use this as a reference to write a code that gradually inc and decs speed 
#this code has a toggle to choose between gradual increase and has a varible of inc in speed

from gpiozero import Motor
from time import sleep

def control_motor_speed(motor, speed, duration):
    motor.forward(speed)
    sleep(duration)
    motor.stop()

def gradual_speed_change(total_duration, gradual_change=True, rate_of_increase=10):
    motor_pin1 = 17  # GPIO pin for motor input 1
    motor_pin2 = 18  # GPIO pin for motor input 2
    motor = Motor(forward=motor_pin1, backward=motor_pin2)

    try:
        if gradual_change:
            # Calculate the number of steps for gradual increase and decrease
            steps = int((total_duration / 2) / 0.5)

            # Gradually increase the speed
            for _ in range(steps):
                speed = 100 * (_ + 1) / steps
                control_motor_speed(motor, speed/100.0, rate_of_increase / 100.0)

            # Hold the maximum speed for the remaining time
            sleep(total_duration - (steps * (rate_of_increase / 100.0) * 2))

            # Gradually decrease the speed
            for _ in range(steps):
                speed = 100 * (steps - _) / steps
                control_motor_speed(motor, speed/100.0, rate_of_increase / 100.0)
        else:
            # Hold a constant speed for the entire duration
            control_motor_speed(motor, 1.0, total_duration)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop the motor when the script is interrupted
        motor.stop()

if __name__ == "__main__":
    total_duration = 10  # Total duration in seconds
    gradual_change = True  # Set to False to hold constant speed
    rate_of_increase = 10  # Adjust as needed
    gradual_speed_change(total_duration, gradual_change, rate_of_increase)
