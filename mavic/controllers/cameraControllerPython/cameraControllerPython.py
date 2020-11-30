"""cameraControllerPython controller."""

'''

Jorge Diego Rodríguez Saltijeral - A01381487
Pablo César Ruíz Hernández - A01197044
Francisco Rodolfo Madero Martin - A00820899
Humberto Tello - A01196965
Saúl Bermea González - A01282698 

'''

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

camera = Camera("eagleCamera")
camera.enable(timestep)
camera.getImage()
camera.saveImage("Image\map.jpg", 100)
#camera.saveImage("C:\Users\saulb\Documents\Saul_Tec\7 semestre\Visión para robots\Proyecto\final\AutonomousDrone-main\mavic_Edit_python\controllers\cameraControllerPython\image.jpg", 100)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    #print("image")
    camera.getImage()
    camera.saveImage("Image\map.jpg", 100)
    #camera.getImage()
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
