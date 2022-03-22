"""
Red Team Goalkeeper robot behaviours.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import (TIME_STEP, Motions)
from Utils import Functions
import RedTeamStrategies

class Goalkeeper (SoccerRobot):
  def run(self):

    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():

        # Do not remove this!
        # ----------------------
        self.getSupervisorData()
        # ----------------------

        # Use the ballData (location) to do something.
        ballCoordinate = self.getBallData()
        # print("RedGoalkeeper - ballCoordinate: ", ballCoordinate)
        selfCoordinate = self.getSelfCoordinate()
        # print("RedGoalkeeper - selfCoordinate: ", selfCoordinate)
        decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)
        # print("RedGoalkeeper - decidedMotion: ", decidedMotion.Name)
        if self.isNewMotionValid(decidedMotion):

          forwardsSprintInterrupt = self.currentlyMoving and (self.currentlyMoving.name == self.motions.forwardsSprint.name and decidedMotion.name != self.motions.forwardsSprint.name)

          # interruptCheck = self.currentlyMoving and\
          #          (self.currentlyMoving.name == self.motions.turnLeft40.name and decidedMotion.name != self.motions.turnLeft40.name and\
          #           decidedMotion.name != self.motions.sideStepLeft.name and decidedMotion.name != self.motions.sideStepRight.name) or\
          #          (self.currentlyMoving.name == self.motions.turnRight40.name and decidedMotion.name != self.motions.turnRight40.name)

          leftShootCheck = self.currentlyMoving and self.currentlyMoving.name == self.motions.rightShoot.name and self.currentlyMoving.isOver() and decidedMotion.name == self.motions.rightShoot.name

          # if interruptCheck:
          #   self.interruptMotion()
          # if forwardsSprintInterrupt:
          #   self.interruptForwardsSprint()
            # print("RedGoalkeeper - Motion interrupted!")
          self.clearMotionQueue()
          # if interruptCheck:
          #   self.addMotionToQueue(self.motions.standInit)
          if leftShootCheck:
            self.addMotionToQueue(self.motions.shoot)
          else:
            self.addMotionToQueue(decidedMotion)
        
        self.startMotion()
      else:

        # It seems there is a problem.
        # print("NO BALL DATA!!!")
        pass

  # Override decideMotion
  def decideMotion(self, ballCoordinate, selfCoordinate):

    # Check the goal scored to balance itself.
    if self.checkGoal() == 1:
      return self.motions.handWave
    elif self.checkGoal() == -1:
      return self.motions.standInit

    # Fall Detection
    robotHeightFromGround = self.getSelfCoordinate()[2]
    if robotHeightFromGround < 0.2:
      if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
        return self.motions.standUpFromBack
      else:
        return self.motions.standUpFromFront

    # Check the oponent has ball priority.
    if self.getBallPriority() == "B":
      return self.motions.standInit

    robotHeadingAngle = self.getRollPitchYaw()[2]

    # Ball at the 2nd zone.
    if RedTeamStrategies.getZone(ballCoordinate) == 2:

      # Find the angle between the ball and robot heading.
      turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
      turningMotion = self.getTurningMotion(turningAngle)
      if turningMotion is not None:
        return turningMotion

      bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

      # Decide wehere to shoot or pass.
      if bodyDistanceFromBall < 0.25:
        # We have to look for the distance from left foot becuase or robots are left footed.
        
        # If decided to shoot
        # We have to calculate the goal angle and sideSteps according to this angle.
        turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.RED_GOAL["Left"], selfCoordinate, robotHeadingAngle)
        turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.RED_GOAL["Right"], selfCoordinate, robotHeadingAngle)
        
        if (turningAngleForGoalLeft > 0 and turningAngleForGoalRight > 0) or (turningAngleForGoalLeft < 0 and turningAngleForGoalRight < 0) or\
            ((abs(turningAngleForGoalLeft) > 90) and (abs(turningAngleForGoalLeft) > abs(turningAngleForGoalRight))) or\
            ((abs(turningAngleForGoalLeft) > 90) and (abs(turningAngleForGoalLeft) < abs(turningAngleForGoalRight))):
          if bodyDistanceFromBall < 0.2:
            return self.motions.rightShoot
          else:
            return self.motions.forwardsSprint
        else:
          return self.motions.sideStepRight
        
      if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
        self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

      return self.motions.forwardsSprint

    # Head to ball.
    else:
      # Find the angle between the ball and robot heading.
      turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
      turningMotion = self.getTurningMotion(turningAngle)
      if turningMotion is not None:
        return turningMotion

    # Stand by.
    return self.motions.standInit
    