import RPi.GPIO as GPIO
from gpiozero import Motor, Servo, Button
from time import sleep, time

# Define pin connections
LEFT_MOTOR_PIN_1 = 2
LEFT_MOTOR_PIN_2 = 3
RIGHT_MOTOR_PIN_1 = 4
RIGHT_MOTOR_PIN_2 = 5
IR_SENSOR_LEFT = 0  # Assuming A0 is connected to an ADC, mapped to GPIO
IR_SENSOR_RIGHT = 1 # Assuming A1 is connected to an ADC, mapped to GPIO
ULTRASONIC_TRIG_PIN = 6
ULTRASONIC_ECHO_PIN = 7
SERVO_PIN = 8
ON_OFF_SWITCH_PIN = 9

# Define thresholds for sensors
IR_THRESHOLD = 700
OBSTACLE_DISTANCE = 20  # Distance in centimeters

# Define servo positions
SERVO_MIDDLE_POSITION = 0.5
SERVO_LEFT_POSITION = 0.0
SERVO_RIGHT_POSITION = 1.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(ULTRASONIC_TRIG_PIN, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_PIN, GPIO.IN)
GPIO.setup(ON_OFF_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

left_motor = Motor(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2)
right_motor = Motor(RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2)
servo_motor = Servo(SERVO_PIN)
on_off_switch = Button(ON_OFF_SWITCH_PIN)

servo_motor.value = SERVO_MIDDLE_POSITION  # Set the initial position of the servo

def get_ir_value(channel):
    # Implement ADC read code here, for example using an ADC library
    return 0  # Placeholder value

def get_ultrasonic_distance():
    GPIO.output(ULTRASONIC_TRIG_PIN, False)
    sleep(0.000002)
    GPIO.output(ULTRASONIC_TRIG_PIN, True)
    sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG_PIN, False)
    
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 0:
        pulse_start = time()
    
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 1:
        pulse_end = time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

def move_forward():
    left_motor.forward()
    right_motor.forward()

def turn_left():
    left_motor.backward()
    right_motor.forward()

def turn_right():
    left_motor.forward()
    right_motor.backward()

def stop_motors():
    left_motor.stop()
    right_motor.stop()

def avoid_obstacles():
    servo_motor.value = SERVO_LEFT_POSITION
    sleep(0.5)
    distance = get_ultrasonic_distance()
    
    if distance > OBSTACLE_DISTANCE:
        move_forward()
    else:
        stop_motors()
        sleep(0.5)
        turn_right()
        sleep(1)
    
    servo_motor.value = SERVO_MIDDLE_POSITION

try:
    while True:
        if not on_off_switch.is_pressed:
            ir_left_value = get_ir_value(IR_SENSOR_LEFT)
            ir_right_value = get_ir_value(IR_SENSOR_RIGHT)
    
            if ir_left_value < IR_THRESHOLD and ir_right_value < IR_THRESHOLD:
                move_forward()
            elif ir_left_value < IR_THRESHOLD:
                turn_left()
            elif ir_right_value < IR_THRESHOLD:
                turn_right()
            else:
                avoid_obstacles()
        else:
            stop_motors()
        sleep(0.1)  # Small delay to avoid high CPU usage
finally:
    GPIO.cleanup()
