"""
Blue Team Main Controller.
This controller should be selected as controller for Blue team robots.
Roles will be assigned automatically for each robot.
"""

from controller import Robot
from BlueGoalkeeper import Goalkeeper
from BlueDefender import Defender
from BlueForwardLeft import ForwardLeft
from BlueForwardRight import ForwardRight

# Create the Robot instance.
robot = Robot()
# Get the Robot Name to find the role.
robotName = robot.getName()

# Compare the Robot Name and assign the role.
if robotName == "BLUE_GK":
    robotController = Goalkeeper(robot)
elif robotName == "BLUE_DEF":
    robotController = Defender(robot)
elif robotName == "BLUE_FW_L":
    robotController = ForwardLeft(robot)
else:
    robotController = ForwardRight(robot)

# Run the Robot Controller.
robotController.run()