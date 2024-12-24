import time
import RPi.GPIO as GPIO
import threading

# Global flag to control servo movement
servo_running = False
current_angle = 0 

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)



# # Allocating Pins
flame_pin_center = 26
flame_pin_left = 6
flame_pin_right = 5
relay_pin = 16      
servo_pin = 18
ultra_trig = 19
ultra_echo = 13

# Wheels
left_forward = 17   # IN1
left_backward = 27  # IN2
right_forward = 22  # IN3
right_backward = 23 # IN4

# # Setup pins
GPIO.setup(flame_pin_center, GPIO.IN)
GPIO.setup(flame_pin_left, GPIO.IN)
GPIO.setup(flame_pin_right, GPIO.IN)
GPIO.setup(relay_pin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(left_forward, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(left_backward, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(right_forward, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(right_backward, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(ultra_trig, GPIO.OUT)
GPIO.setup(ultra_echo, GPIO.IN)

# Set up PWM on servo-pin at 50Hz (standard for most servos)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)  # Start PWM with 0% duty cycle


  #### L298N CODE FOR WHEELS 
# Functions to control the motors
def move_forward():
  GPIO.output(left_forward, GPIO.LOW)
  GPIO.output(left_backward, GPIO.HIGH)
  GPIO.output(right_forward, GPIO.LOW)
  GPIO.output(right_backward, GPIO.HIGH)

def move_backward():
  GPIO.output(left_forward, GPIO.HIGH)
  GPIO.output(left_backward, GPIO.LOW)
  GPIO.output(right_forward, GPIO.HIGH)
  GPIO.output(right_backward, GPIO.LOW)
    
def turn_left():
  GPIO.output(left_forward, GPIO.HIGH)
  GPIO.output(left_backward, GPIO.LOW)
  GPIO.output(right_forward, GPIO.LOW)
  GPIO.output(right_backward, GPIO.HIGH)

def turn_right():
  GPIO.output(left_forward, GPIO.LOW)
  GPIO.output(left_backward, GPIO.HIGH)
  GPIO.output(right_forward, GPIO.HIGH)
  GPIO.output(right_backward, GPIO.LOW)

def stop():
  GPIO.output(left_forward, GPIO.LOW)
  GPIO.output(left_backward, GPIO.LOW)
  GPIO.output(right_forward, GPIO.LOW)
  GPIO.output(right_backward, GPIO.LOW)

  #### UltraSonic Code
def get_distance():
  # Send a 10µs pulse to trigger
  GPIO.output(ultra_trig, True)
  time.sleep(0.00001)
  GPIO.output(ultra_trig, False)

  # Measure the time for the ECHO signal
  start_time = time.time()
  while GPIO.input(ultra_echo) == 0:
    start_time = time.time()

  while GPIO.input(ultra_echo) == 1:
    end_time = time.time()

  duration = end_time - start_time
  distance = duration * 17000  # for cm
  return distance
  

  #### RELAY CODE FOR WATER PUMP
def pump_on():
  if not servo_running:  # Only start servo if it's not already running
      GPIO.output(relay_pin, GPIO.LOW)  # Activate relay
      # Start the servo scanning in a separate thread
      threading.Thread(target=start_servo_scan, daemon=True).start()

def pump_off():
  GPIO.output(relay_pin, GPIO.HIGH) # Deactivate relay
  stop_servo_scan()                 # Stop servo movement

  #### SERVO CODE
  
def set_angle(angle):
  # Constrain the angle to -60 to 60 degrees
  angle = max(-60, min(60, angle))
  duty = 2 + (angle + 90) * (10 / 180)  # Convert angle to duty cycle for PWM
  pwm.ChangeDutyCycle(duty)
  time.sleep(0.1)  # Give time for the servo to move
  pwm.ChangeDutyCycle(0)  # Stop the PWM signal to hold the position
  global current_angle
  current_angle = angle

def start_servo_scan():
  global servo_running
  if servo_running:  # If the servo is already running, do not start again
      return

  servo_running = True
  global current_angle
  
  direction = 1  # 1 for moving towards +60, -1 for moving back to -60
  
  while servo_running:
    set_angle(current_angle)
    # Update the angle
    current_angle += 5 * direction  # Move 5 degrees per step (slow movement)
        
    if current_angle >= 60:  # If the angle reaches +60, reverse direction
        current_angle = 60  # Cap at +60
        direction = -1  # Reverse direction to move towards -60
        
    elif current_angle <= -60:  # If the angle reaches -60, reverse direction
        current_angle = -60  # Cap at -60
        direction = 1  # Reverse direction to move towards +60

    time.sleep(0.1)  # Control the speed of movement (slower movement)

    
def stop_servo_scan():
  global servo_running
  servo_running = False
  set_angle(0)  # Move the servo to 0 degrees and stop
  print("Servo stopped at 0 degrees.")


try:
  while True:
    # Measure distance from ultrasonic sensor
    dist = get_distance()
    print(f"Object detected at distance: {dist:.2f} cm")

    # Read flame sensor states
    flame_center = GPIO.input(flame_pin_center)
    flame_left = GPIO.input(flame_pin_left)
    flame_right = GPIO.input(flame_pin_right)
    print(f"Flame Status -> Center: {flame_center}, Left: {flame_left}, Right: {flame_right}")
    

    # Stop the car if any object (or fire) is within 30 cm
    if dist <= 35:
      stop()
      print("Car Stopped! Object too close.")
            
      # Check if fire is detected at this point
      if flame_center == GPIO.HIGH or flame_left == GPIO.LOW or flame_right == GPIO.LOW:
        print("Fire detected nearby!")
        pump_on()  # Activate water pump
        time.sleep(10)  # Keep pump active for 5 second
        pump_off()  # Deactivate water pump
      continue  # Skip further movement logic for this iteration

    # Movement logic based on flame sensor inputs
    if flame_center == GPIO.HIGH:  # Fire detected directly in front
      move_forward()
      print("Moving forward towards fire!")
    elif flame_left == GPIO.LOW:  # Fire detected to the left
      turn_left()
      print("Turning left towards fire!")
    elif flame_right == GPIO.LOW:  # Fire detected to the right
      turn_right()
      print("Turning right towards fire!")

    time.sleep(0.2)  # Small delay to stabilize the loop
        
except KeyboardInterrupt:
    print("Gracefully Exiting....")
    
finally:
    GPIO.cleanup()
    pwm.stop()