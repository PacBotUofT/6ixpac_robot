from gpiozero import Motor, RotaryEncoder
from time import sleep
 
# Motor driver connections
motor = Motor(forward=17, backward=18, enable=22) 
 
# Quadrature encoder connections
encoderA = RotaryEncoder(23, 24, max_steps=90)
encoderA.value = 0
timesSpun = 0.0

wheelDiameter = 1.825
wheelCircumf = 3.141 * wheelDiameter

fullRotation = False

def distanceRolled(timesSpun):
    return timesSpun * wheelCircumf

# Run the motor forward for 3 seconds
#print("Running motor forward...")
#motor.forward()
#sleep(3)

# Stop the motor and print encoder values
#motor.stop()
#print_encoder_values()
#sleep(1)

# Run the motor backward for 3 seconds
#print("Running motor backward...")
#motor.backward()
#sleep(3)

# Stop the motor and print encoder values
#motor.stop()
#print_encoder_values()
while True:
    if abs(encoderA.value) == 1.0 and (not fullRotation):
        print("condition met")
        timesSpun = 1.0
        encoderA.value = 0.0
        fullRotation = True
    timesSpun = (1 + abs(encoderA.value)) if fullRotation else abs(encoderA.value)
    print(f"# of rotations: {timesSpun}, distance rolled: {distanceRolled(timesSpun)}, encoder val: {encoderA.value}, fullRotation: {fullRotation}")

    if(distanceRolled(timesSpun) >= 7):
        # Stop the motor on keyboard interrupt (Ctrl+C)
        motor.stop()
        motor.close()
        encoderA.close()
        exit()
