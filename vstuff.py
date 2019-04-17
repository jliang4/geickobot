import math
import time
import sys
import syslog
import random
import RPi.GPIO as GPIO
import smbus
import logging
import keyboard
from Adafruit_BNO055 import BNO055

BNO_AXIS_REMAP = { 'x': BNO055.AXIS_REMAP_X,
                   'y': BNO055.AXIS_REMAP_Z,
                   'z': BNO055.AXIS_REMAP_Y,
                   'x_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'y_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'z_sign': BNO055.AXIS_REMAP_NEGATIVE }

#19,12,5,21
#22,16
#23,24
#20 DOES NOT WORK?
CALIBRATION_FILE=""
LEFTF=12
LEFTB=19
LEFTEN=22
RIGHTF=5
RIGHTB=21
RIGHTEN=16
ARDUINO1=23
ARDUINO2=24
MUXADDR=0x70
PROXADDR=0x13

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFTF, GPIO.OUT)
GPIO.setup(LEFTB, GPIO.OUT)
GPIO.setup(LEFTEN, GPIO.OUT)
GPIO.setup(RIGHTF, GPIO.OUT)
GPIO.setup(RIGHTB, GPIO.OUT)
GPIO.setup(RIGHTEN, GPIO.OUT)
GPIO.setup(ARDUINO1, GPIO.OUT)
GPIO.setup(ARDUINO2, GPIO.OUT)
pwmL = GPIO.PWM(LEFTEN, 500)
pwmR = GPIO.PWM(RIGHTEN, 500)
pwmL.start(0)
pwmR.start(0)

bus = smbus.SMBus(1)
proximityThreshold=2300

#Kp, Ki, Kd
kRotate = [4,0,0.5]
ffRotate = 0
kStraight = [1,0,0]
ffStraight = 50
interval = 0.05
state = "M"
stack = []
pathStage = "F" #F-Forward, B-Backward, D-down, U-up, FE-forward to edge, BE-backward to edge
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))

print('Reading BNO055 data, press Ctrl-C to quit...')

def load_calibration():
	global CALIBRATION_FILE
	global bno

    # Load calibration from disk.
    with open(CALIBRATION_FILE, 'r') as cal_file:
        data = json.load(cal_file)

    # Grab the lock on BNO sensor access to serial access to the sensor.
    with bno_changed:
        bno.set_calibration(data)
    return 'OK'

def setSpeeds(L,R):
	global pwmL
	global pwmR

	pwmL.ChangeDutyCycle(L)
	pwmR.ChangeDutyCycle(R)

def setDirection(L,R):
	global LEFTF
	global LEFTB
	global LEFTEN
	global RIGHTF
	global RIGHTB
	global RIGHTEN

	if L:
		GPIO.output(LEFTF,GPIO.HIGH)
		GPIO.output(LEFTB,GPIO.LOW)
	else:
		GPIO.output(LEFTF,GPIO.LOW)
		GPIO.output(LEFTB,GPIO.HIGH)

	if R:
		GPIO.output(RIGHTF,GPIO.HIGH)
		GPIO.output(RIGHTB,GPIO.LOW)
	else:
		GPIO.output(RIGHTF,GPIO.LOW)
		GPIO.output(RIGHTB,GPIO.HIGH)

def stopWheels():
		GPIO.output(LEFTF,GPIO.LOW)
		GPIO.output(LEFTB,GPIO.LOW)
		GPIO.output(RIGHTF,GPIO.LOW)
		GPIO.output(RIGHTB,GPIO.LOW)

def setPropellors(A1,A2):
	global ARDUINO1
	global ARDUINO2

	if A1:
		GPIO.output(ARDUINO1,GPIO.HIGH)
	else:
		GPIO.output(ARDUINO1,GPIO.LOW)

	if A2:
		GPIO.output(ARDUINO2,GPIO.HIGH)
	else:
		GPIO.output(ARDUINO2,GPIO.LOW)

def getProximity(direction):
	global bus
	global MUXADDR
	global PROXADDR

	bus.write_byte(MUXADDR,2**direction)
	bus.write_byte_data(PROXADDR,0x80,10)

	return bus.read_byte_data(PROXADDR,0x87)*128+bus.read_byte_data(PROXADDR,0x88)

def getCloseProximity():
	r = []
	for i in range(0,8):
		try:
			if getProximity(i) > proximityThreshold:
				r.append(i)
		except:
			print("Proximity Sensor " + str(i) + " error!")
	return r

def goAngle(pitch):
	global state
	global kRotate
	global ffRotate

	setSpeeds(0,0)
	stopWheels()

	Kp = KRotate[0]
	Ki = KRotate[1]
	Kd = KRotate[2]

	lastError = 0
	totalError = 0

	Cheading, Croll, Cpitch = bno.read_euler()
	error = Cpitch - pitch

	while state == "V" and abs(error) > 1:
		Cheading, Croll, Cpitch = bno.read_euler()
		error = Cpitch - pitch
		speed = Kp*error + Kd*lastError/interval + Ki*totalError + ffRotate
		if speed > 0:
			setDirection(1,0)
		else:
			setDirection(0,1)
		speed = min(100,abs(speed))
		setSpeeds(speed,speed)
		lastError = error
		totalError += error*interval
		time.sleep(interval)

	stopWheels()

def maintainAngle(pitch,direction):
	global state
	global kStraight
	global ffStraight

	setSpeeds(0,0)
	setDirection(direction,direction)

	Kp = kStraight[0]
	Ki = kStraight[1]
	Kd = kStraight[2]

	lastError = 0
	totalError = 0

	Cheading, Croll, Cpitch = bno.read_euler()
	error = Cpitch - pitch

	while state == "V" and len(getCloseProximity()) == 0:
		Cheading, Croll, Cpitch = bno.read_euler()
		error = Cpitch - pitch
		speed = Kp*error + Kd*lastError/interval + Ki*totalError + ffStraight
		setSpeeds(ffStraight,min(100,abs(speed)))
		lastError = error
		totalError += error*interval
		time.sleep(interval)

	stopWheels()

def changeRow(horizontal,vertical):
	return 1

def handleKeyPress(key):
	global stack
	global state

	#Manual control mode and keypress handlers
	if key.name == "M":
		state = "M"
		stack = []
		stopWheels()
	elif state == "M" and key.name == "p":
		setPropellors(1,1)
	elif state == "M" and key.name == "k":
		setPropellors(1,0)
	elif state == "M" and key.name == "l":
		setPropellors(0,1)
	elif state == "M" and key.name == "o":
		setPropellors(0,0)
	elif state == "M" and key.name in "wasd":
		if key.name not in stack:
			stack.append(key.name)
	elif state == "M" and key.name in  "01234567879":
		setSpeeds(100.0*(int(key.name)/9))

	#Sensor testing mode
	elif key.name == "n":
		state = "N"

	#Autonomous mode
	elif key.name == "v":
		state = "V"

#TODO: I think this goes here
bno.set_axis_remap(**BNO_AXIS_REMAP)
load_calibration()

time.sleep(1)
# Read the Euler angles for heading, roll, pitch (all in degrees).
heading, roll, pitch = bno.read_euler()
# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
sys, gyro, accel, mag = bno.get_calibration_status()
# Print everything out.
print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
      heading, roll, pitch, sys, gyro, accel, mag))
# Other values you can optionally read:
# Orientation as a quaternion:
#x,y,z,w = bno.read_quaterion()
# Accelerometer data (in meters per second squared):
#x,y,z = bno.read_accelerometer()
# Linear acceleration data (i.e. acceleration from movement, not gravity--
# returned in meters per second squared):
#x,y,z = bno.read_linear_acceleration()

time.sleep(1)

keyboard.on_press(handleKeyPress)

try:
	while True:
		#Manual Control Mode
		while state == "M":
			for c in "wasd":
				if not keyboard.is_pressed(c):
					stack.remove(c)
			if len(stack) == 0:
				stopWheels()
			else:
				c = stack[-1]
				if c == "w":
					setDirection(1,1)
				elif c == "a":
					setDirection(0,1)
				elif c == "s":
					setDirection(1,0)
				else:
					setDirection(0,0)
		#Sensor Test Mode
		if state == "N":
			stopWheels()
			setPropellors(1,1)
			while state == "N":
				Cheading, Croll, Cpitch = bno.read_euler()
				print("Heading: {0:.2f}, Roll: {1:.2f}, Pitch: {2:.2f}".format(Cheading, Croll, Cpitch))
				#print("Heading: {0:.2f}, Roll: {1:.2f}, Pitch: {2:.2f}".format(bno.read_euler()))
				print("Threshold Passed: " + str(getCloseProximity()))			
				time.sleep(interval)

		#Autonomous Mode
		if state == "V":
			goAngle(90)
			while state == "V":
				time.sleep(1) #Actual code goes here


except KeyboardInterrupt:
	keyboard.unhook_all()
	GPIO.cleanup()