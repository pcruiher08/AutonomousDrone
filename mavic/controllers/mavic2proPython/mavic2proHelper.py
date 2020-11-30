def getMotorAll(robot):
	frontLeftMotor = robot.getMotor('front left propeller')
	frontRightMotor = robot.getMotor('front right propeller')
	backLeftMotor = robot.getMotor('rear left propeller')
	backRightMotor = robot.getMotor('rear right propeller')
	return [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]

def motorsSpeed(robot, v1, v2, v3, v4):
	[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
	frontLeftMotor.setVelocity(v1)
	frontRightMotor.setVelocity(v2)
	backLeftMotor.setVelocity(v3)
	backRightMotor.setVelocity(v4)
	return

def initialiseMotors(robot, MAX_PROPELLER_VELOCITY):
	[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
	frontLeftMotor.setPosition(float('inf'))
	frontRightMotor.setPosition(float('inf'))
	backLeftMotor.setPosition(float('inf'))
	backRightMotor.setPosition(float('inf'))

	motorsSpeed(robot, 0, 0, 0, 0)
	return


'''
recuperado de internet por
Jorge Diego Rodríguez Saltijeral - A01381487
Pablo César Ruíz Hernández - A01197044
Francisco Rodolfo Madero Martin - A00820899
Humberto Tello - A01196965
Saúl Bermea González - A01282698 

'''