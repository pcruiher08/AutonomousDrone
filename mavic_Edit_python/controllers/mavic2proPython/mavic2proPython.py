from controller import *
import mavic2proHelper
import image_processing
import PID
import csv
import struct
import numpy as np
from timeit import default_timer as timer
import time
import math
#C:\Users\saulb\AppData\Local\Programs\Python\Python37\python.exe
#C:\Python38\python.exe
#C:\ProgramData\Anaconda3\pythonw.exe
#python -m ensurepip --default-pip

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]



TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433

robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led = LED("front left led")
front_right_led = LED("front right led")
gps = GPS("gps")
gps.enable(TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)
#cameraRobot = Robot("cameraRobot")
#camera = cameraRobot.getCamera("eagleCamera")
#camera = Camera("eagleCamera")
#camera.enable(TIME_STEP)
#camera.getImage()
#image = camera.getImage()
#data = np.array(image.getdata(), np.uint8).reshape(image.size[1], image.size[0], 3)

yaw_setpoint=-1
pitch_Kp = 2
pitch_Ki = 0.1
pitch_Kd = 2
roll_Kp = 2
roll_Ki = 0.1
roll_Kd = 2
pitchPID = PID.PID(float(pitch_Kp), float(pitch_Ki), float(pitch_Kd), setpoint=0.0)
rollPID = PID.PID(float(roll_Kp), float(roll_Ki), float(roll_Kd), setpoint=0.0)
throttlePID = PID.PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=0.5)
yawPID = PID.PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(yaw_setpoint))

def headTo(targetX, targetY, targetZ, maxTimeToGetThere):
	startTime = timer()
	print([targetX, targetY, targetZ])
	while (robot.step(timestep) != -1 and (timer() - startTime < maxTimeToGetThere)):
		#print("im flying")
		led_state = int(robot.getTime()) % 2
		front_left_led.set(led_state)
		front_right_led.set(int(not(led_state)))

		roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
		pitch = imu.getRollPitchYaw()[1]
		yaw = compass.getValues()[0]
		roll_acceleration = gyro.getValues()[0]
		pitch_acceleration = gyro.getValues()[1]
		
		xGPS = gps.getValues()[2]
		yGPS = gps.getValues()[0]
		zGPS = gps.getValues()[1]
		throttlePID.setpoint = targetZ
		vertical_input = throttlePID(zGPS)
		yaw_input = yawPID(yaw)
		
		rollPID.setpoint = targetX
		pitchPID.setpoint = targetY
		
		roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
		pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(-yGPS)

		front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
		front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
		rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
		rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input

		mavic2proHelper.motorsSpeed(robot, front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
	
	#print("im not flying anymore")
	return
#xSol = np.array([0.258552, -18.4218, -26.1784, -45.3112, -61.4061])
#ySol = np.array([-0.261272, -2.48208, 0.261272, 5.61734, 22.6])

##xSol = np.array([  0.258125, -18.4525  , -26.209384, -45.34774 , -61.405593])
##ySol = np.array([ -0.259677,  -2.397053,   0.347221,   5.718543,  22.599245])

xSol, ySol = image_processing.process()
Sol = np.concatenate(([xSol], [ySol]), axis=0)
Sol = np.concatenate((Sol, np.ones((1, np.size(Sol, axis=1)))), axis=0)
Sol = np.concatenate((np.array([[0,0],[0,0],[0.1, 1]]), Sol), axis=1)
Sol = np.concatenate((Sol, np.array([[Sol[0, -1]],[Sol[1, -1]],[0.1]])), axis=1)
print("---Solution---")
print(Sol)
#headTo(0,0.25,0,6)
#headTo(0,0.5,0,6)
#headTo(0,0.75,0,6)
#headTo(0,1,0,6)
#headTo(0,1.2,0,6)
speed=0.05
i=1
while i<np.size(Sol, 1):
    distance =  math.sqrt((Sol[0, i]-Sol[0, i-1])**2+(Sol[1, i]-Sol[1, i-1])**2+(Sol[2, i]-Sol[2, i-1])**2)
    if(i==1):
        time =distance/0.02
    else:
        time =distance/speed
    div = distance//0.1 +1
    if (time<0.5):
        time = 0.8
    for j in range(round(div)):
        xStep = (1-j/div)*Sol[0, i-1]+(j/div)*Sol[0, i]
        yStep = (1-j/div)*Sol[1, i-1]+(j/div)*Sol[1, i]
        zStep = (1-j/div)*Sol[2, i-1]+(j/div)*Sol[2, i]
        headTo(yStep,-xStep,zStep,time/div)#(xStep,zStep,yStep,time)
    i=i+1
#headTo(2.0,1.5,1,2.4)
#headTo(2.0,1,1,2.4)
#headTo(1,1,1,2.4)
#headTo(0,1,1,2.4)
#headTo(0,0,1,2.4)
#headTo(0,0,0,1000)