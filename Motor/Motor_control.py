from gpiozero import Motor, RotaryEncoder
import time
import threading

# Motor driver connections
motorN = Motor(forward=17, backward=27)
motorE = Motor(forward=10, backward=9)
motorS = Motor(forward=0, backward=5)
motorW = Motor(forward=13, backward=19)

# Quadrature encoder connections
encoderN = RotaryEncoder(14, 15, max_steps=2500, wrap=True)
encoderE = RotaryEncoder(23, 24, max_steps=2500, wrap=True)
encoderS = RotaryEncoder(1, 12, max_steps=2500, wrap=True)
encoderW = RotaryEncoder(16, 20, max_steps=2500, wrap=True)

wheel_diameter = 1.850394
wheel_circumf = 3.141593 * wheel_diameter
steps_per_rev = 90
steps_per_block = (7 * steps_per_rev) / wheel_circumf

# Global variable and lock for thread-safe interrupt handling
interrupt_flag = False
interrupt_lock = threading.Lock()


def control_motor_speed(direction, speed):
    if direction == 'N':
        motorE.backward(speed)
        motorW.forward(speed)
    if direction == 'S':
        motorE.forward(speed)
        motorW.backward(speed)
    if direction == 'E':
        motorN.forward(speed)
        motorS.backward(speed)
    if direction == 'W':
        motorN.backward(speed)
        motorS.forward(speed)


def gradual_speed_change(target_steps, direction, encoder, gradual_change=True, rate_of_increase=4):
    global interrupt_flag

    speed = 0.0

    try:
        if gradual_change:
            while abs(encoder.steps) < (target_steps / (2 * rate_of_increase)):
                with interrupt_lock:
                    if interrupt_flag:
                        print("Interrupting gradual_speed_change")
                        return
                speed = min(speed + (rate_of_increase / 100.0), 1.0)
                control_motor_speed(direction, speed)
                print(abs(encoder.steps))
                time.sleep(0.1)

            while abs(encoder.steps) < (target_steps * (1 - (1 / (2 * rate_of_increase)))):
                with interrupt_lock:
                    if interrupt_flag:
                        print("Interrupting gradual_speed_change")
                        return
                control_motor_speed(direction, 1.0)
                print(abs(encoder.steps))
                time.sleep(0.1)

            while abs(encoder.steps) < target_steps:
                with interrupt_lock:
                    if interrupt_flag:
                        print("Interrupting gradual_speed_change")
                        return
                speed = max(speed - (rate_of_increase / 100.0), 0.25)
                control_motor_speed(direction, speed)
                print(abs(encoder.steps))
                time.sleep(0.1)
        else:
            while abs(encoder.steps) < target_steps:
                with interrupt_lock:
                    if interrupt_flag:
                        print("Interrupting gradual_speed_change")
                        return
                control_motor_speed(direction, 1.0)

    except KeyboardInterrupt:
        pass


def move_robot(num_blocks, direction, acceleration):
    global interrupt_flag

    with interrupt_lock:
        interrupt_flag = False

    encoderN.steps = 0
    encoderE.steps = 0

    target_steps = int(steps_per_block * num_blocks)

    print(f"Moving {num_blocks} block(s) {direction} with acceleration {acceleration}")
    print(f"Target steps: {target_steps}")

    try:
        if direction == 'N' or direction == 'S':
            gradual_speed_change(target_steps, direction,
                                 encoderE, rate_of_increase=acceleration)
        if direction == 'E' or direction == 'W':
            gradual_speed_change(target_steps, direction,
                                 encoderN, rate_of_increase=acceleration)
    finally:
        print("Finished moving")
        encoderN.steps = 0
        encoderE.steps = 0

        with interrupt_lock:
            interrupt_flag = False  # Reset the interrupt flag

# Interrupt function


def interrupt_movement():
    with interrupt_lock:
        global interrupt_flag
        interrupt_flag = True


# Example usage:
# Start the motor control in a separate thread
motor_thread = threading.Thread(target=move_robot, args=(5, 'N', 4))
motor_thread.start()

# Simulate an interrupt after 2 seconds
time.sleep(2)
interrupt_movement()

# Wait for the motor control thread to finish
motor_thread.join()
