from controller import Motion
import math
from enum import Enum

TIME_STEP = 40

PI = math.pi

INITIAL_TRANSLATIONS = {
  "BALL"      : [ 0.00,  0.00, 0.0798759],
  "RED_GK"    : [-4.00,  0.00, 0.33],
  "RED_DEF_L" : [-2.50,  1.20, 0.33],
  "RED_DEF_R" : [-2.50, -1.20, 0.33],
  "RED_FW"    : [-0.80,  0.00, 0.33],
  "BLUE_GK"   : [ 4.00,  0.00, 0.33],
  "BLUE_DEF"  : [ 3.00,  0.00, 0.33],
  "BLUE_FW_L"  : [ 1.40, -1.20, 0.33],
  "BLUE_FW_R"  : [ 1.40,  1.20, 0.33]
}

INITIAL_ROTATIONS = {
  "BALL"      : [0, 1, 0, 0.0],
  "RED_GK"    : [0, 1, 0, 0.1],
  "RED_DEF_L" : [0, 1, 0, 0.1],
  "RED_DEF_R" : [0, 1, 0, 0.1],
  "RED_FW"    : [0, 1, 0, 0.1],
  "BLUE_GK"   : [0, 1, 0, 0.1],
  "BLUE_DEF"  : [0, 1, 0, 0.1],
  "BLUE_FW_L"  : [0, 1, 0, 0.1],
  "BLUE_FW_R"  : [0, 1, 0, 0.1]
}

BALL_POSITIONS = {
  "OUT_R"     : [-3.65, 0, 0.0798759],
  "OUT_B"     : [ 3.65, 0, 0.0798759]
}

class Motions:
  def __init__(self):
    self.handWave = MotionBase('handWave', '../../motions/HandWave.motion')
    self.forwards = MotionBase('forwards', '../../motions/Forwards')
    self.forwardsSprint = MotionBase('forwardsSprint', '../../motions/ForwardsSprint.motion')
    self.forwards50 = MotionBase('forwards50', '../../motions/Forwards50.motion')
    self.backwards = MotionBase('backwards', '../../motions/Backwards.motion')
    self.shoot = MotionBase('shoot', '../../motions/Shoot.motion')
    self.rightShoot = MotionBase('rightShoot', '../../motions/RightShoot.motion')
    self.longShoot = MotionBase('longShoot', '../../motions/LongPass.motion')
    self.leftSidePass = MotionBase('leftSidePass','../../motions/SidePass_Left.motion')
    self.rightSidePass = MotionBase('rightSidePass','../../motions/SidePass_Right.motion')
    self.sideStepLeft = MotionBase('sideStepLeft', '../../motions/SideStepLeft.motion')
    self.sideStepRight = MotionBase('sideStepRight', '../../motions/SideStepRight.motion')
    self.standUpFromFront = MotionBase('standUpFromFront', '../../motions/StandUpFromFront.motion')
    self.standUpFromBack = MotionBase('standUpFromBack', '../../motions/StandUpFromBack.motion')
    self.turnLeft10 = MotionBase('turnLeft10', '../../motions/TurnLeft10.motion')
    self.turnLeft20 = MotionBase('turnLeft20', '../../motions/TurnLeft20.motion')
    self.turnLeft30 = MotionBase('turnLeft30', '../../motions/TurnLeft30.motion')
    self.turnLeft40 = MotionBase('turnLeft40', '../../motions/TurnLeft40.motion')
    self.turnLeft60 = MotionBase('turnLeft60', '../../motions/TurnLeft60.motion')
    self.turnLeft180 = MotionBase('turnLeft180', '../../motions/TurnLeft180.motion')
    self.turnRight10 = MotionBase('turnRight10', '../../motions/TurnRight10.motion')
    self.turnRight10_V2 = MotionBase('turnRight10', '../../motions/TurnRight10_V2.motion')
    self.turnRight40 = MotionBase('turnRight40', '../../motions/TurnRight40.motion')
    self.turnRight60 = MotionBase('turnRight60', '../../motions/TurnRight60.motion')
    self.standInit = MotionBase('standInit', '../../motions/StandInit.motion')

class MotionBase(Motion):
  def __init__(self, name, path):
    super().__init__(path)
    self.name = name