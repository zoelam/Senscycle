import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar
import RPi.GPIO as GPIO

# For handling the case where we have to kill the process
# using `pkill -2 -if test.py` or similar
import signal

GPIO.setmode(GPIO.BCM) # Setting up GPIO pins on the board
GPIO.setwarnings(False)

Right_Speaker =18
Right_Vibe = 12
Left_Speaker = 13
Left_Vibe = 19

GPIO.setup(18, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

Right_Speaker = GPIO.PWM(18,3000) #giving output a freq of 3kHz
Right_Vibe = GPIO.PWM(12, 30) #giving output a freq of 30Hz
Left_Speaker = GPIO.PWM(13,3000)
Left_Vibe = GPIO.PWM(19,30)

# If we run into any issues opening the window
CAN_DRAW_WINDOW = False

# Set up pygame and the display
try:
    os.putenv('SDL_FBDEV', '/dev/fb1')
    pygame.init()
    lcd = pygame.display.set_mode((320,240))
    pygame.mouse.set_visible(False)
    lcd.fill((0,0,0))
    pygame.display.update()
    CAN_DRAW_WINDOW = True
except pygame.error:
    CAN_DRAW_WINDOW = False

# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# used to scale data to fit on the screen
max_distance = 0

#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance

    if CAN_DRAW_WINDOW:
        lcd.fill((0,0,0))

    for angle in range(360): # 360 degree scan
        distance = data[angle]
        if distance > 0:         # ignore initially ungathered data points
            max_distance = max([min([11000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))

            if CAN_DRAW_WINDOW:
                lcd.set_at(point, pygame.Color(255, 255, 255)) # display update

            if (distance <= 9144):
                if (angle >= 355 and angle < 358): # Left side detection
                    GPIO.output(13, 1)
                    GPIO.output(19, 1)
                    Left_Speaker.start(50) # setting up duty cycle
                    Left_Vibe.start(50)
                    return
                elif (angle >= 358 and angle < 0): # Center left detection
                    GPIO.output(13, 1)
                    GPIO.output(19, 1)
                    Left_Speaker.start(50) # setting up duty cycle
                    Left_Vibe.start(50)
                    GPIO.output(18, 1)
                    GPIO.output(12, 1)
                    Right_Speaker.start(50) # setting up duty cycle
                    Right_Vibe.start(50)
                    return
                elif (angle >= 0 and angle < 2): # Center right detection
                    GPIO.output(13, 1)
                    GPIO.output(19, 1)
                    Left_Speaker.start(50) # setting up duty cycle
                    Left_Vibe.start(50)
                    GPIO.output(18, 1)
                    GPIO.output(12, 1)
                    Right_Speaker.start(50) # setting up duty cycle
                    Right_Vibe.start(50)
                    return
                elif (angle >= 2 and angle < 5): # Right side detection
                    GPIO.output(18, 1)
                    GPIO.output(12, 1)
                    Right_Speaker.start(50) #setting up duty cycle
                    Right_Vibe.start(50)
                    return
                else: # Keeps haptics low when not in range
                    GPIO.output(13, 0)
                    GPIO.output(19, 0)
                    Left_Speaker.start(0) #setting up duty cycle
                    Left_Vibe.start(0)
                    GPIO.output(18, 0)
                    GPIO.output(12, 0)
                    Right_Speaker.start(0) #setting up duty cycle
                    Right_Vibe.start(0)
            point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))

            if CAN_DRAW_WINDOW:
                lcd.set_at(point, pygame.Color(255, 255, 255))

    if CAN_DRAW_WINDOW:
        pygame.display.update()

scan_data = [0]*360

def lidar_cleanup(a,b):
    print("cleaning up...")
    lidar.stop()
    lidar.disconnect()
    os._exit()

try:
    # install signal handlers for graceful exit
    signal.signal(signal.SIGINT, lidar_cleanup)  # pkill -2 -if test.py (OR `pkill -INT -if test.py`)
    signal.signal(signal.SIGTERM, lidar_cleanup)  # pkill -15 -if test.py  (OR `pkill -TERM -if test.py`)
    signal.signal(signal.SIGHUP, lidar_cleanup)  # pkill -1 -if test.py  (OR `pkill -HUP -if test.py`)

    print(lidar.info)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
except KeyboardInterrupt:
    print('Stoping.')
finally:
    lidar_cleanup(None,None)
