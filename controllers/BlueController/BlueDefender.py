"""
Blue Team Defender robot behaviours.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import (TIME_STEP, Motions)
from controller import Supervisor

class Defender (SoccerRobot):
  def run(self):
    self.printSelf()
    count_0=0
    flag = 0
    flag1 =0
    fixedCoordinate = [3.1, -0.00573, 0.342]
    origin = [0.0723,-0.0798,0.0799]
    goto_Coordinate=[0,0,0]
    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():
        self.getSupervisorData()
        # Use the ballData (location) to do something.
        data = self.supervisorData
        ballOwner = self.getBallOwner()
        ballCoordinate = self.getBallData()
        blue_fw_l = [data[30],data[31],data[32]]
        blue_fw_r = [data[33],data[34],data[35]]
        redFw = [data[21],data[22],data[23]]
        # Get self coordinates
        selfCoordinate = self.getSelfCoordinate()

        # Check the goal scored to balance itself.
        if self.checkGoal() == 1:
          decidedMotion =  self.motions.handWave

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()
          
        elif self.checkGoal() == -1:
          decidedMotion =  self.motions.standInit

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()

        # Check whether the robot falls down.
        robotHeightFromGround = selfCoordinate[2]

        if robotHeightFromGround < 0.2:
          if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
            decidedMotion = self.motions.standUpFromBack
          else:
            decidedMotion = self.motions.standUpFromFront

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()

        # Check the oponent has ball priority.
        elif self.getBallPriority() == "R":
          decidedMotion = self.motions.standInit

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()

        #Approach the ball only in penalty area
        else:
          if ballCoordinate[0]>=2.54 and ballCoordinate[0]<=4.44:
              flag=1
              if ballCoordinate[1]>=-1.5 and ballCoordinate[1]<=1.5 and (ballOwner=='BLUE_GK' or ballOwner[0]=='R'):
                print('going to designated coordinates')
                goto_Coordinate[0]=4.22
                goto_Coordinate[1]=-0.22
                goto_Coordinate[2]=0.315
                decidedMotion = self.decideMotion(ballCoordinate,selfCoordinate,blue_fw_l,blue_fw_r,redFw)            # print("RedForward - decidedMotion: ", decidedMotion.Name)
                
                if self.isNewMotionValid(decidedMotion):
                  boolean = self.currentlyMoving and\
                        (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                  if boolean:
                    self.interruptMotion()
                  self.clearMotionQueue()
                  if boolean:
                    self.addMotionToQueue(self.motions.standInit)
                  self.addMotionToQueue(decidedMotion)
              
                self.startMotion()
              else:    
                  decidedMotion = self.decideMotion(ballCoordinate,selfCoordinate,blue_fw_l,blue_fw_r,redFw)
                  if count_0>=2:
                    decidedMotion=self.motions.rightShoot
                    count_0=0
                  if decidedMotion ==  self.motions.longShoot:
                    count_0=count_0+1
      
                  if self.isNewMotionValid(decidedMotion):
                      boolean = self.currentlyMoving and\
                            (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                      if boolean:
                          self.interruptMotion()
                      self.clearMotionQueue()
                      if boolean:
                          self.addMotionToQueue(self.motions.standInit)
                      self.addMotionToQueue(decidedMotion)
              
                  self.startMotion()
                      
          else:
              if (ballCoordinate[0]<=2.54 or ballCoordinate[0]>=4.44) and flag==1:
                  flag1=0
                  decidedMotion = self.returnMotion(fixedCoordinate,selfCoordinate)
                  if self.isNewMotionValid(decidedMotion):
                      boolean = self.currentlyMoving and\
                        (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                      if boolean:
                          self.interruptMotion()
                      self.clearMotionQueue()
                      if boolean:
                          self.addMotionToQueue(self.motions.standInit)
                      self.addMotionToQueue(decidedMotion)
          
                  self.startMotion()
                  if (selfCoordinate[0]>=2.8 and selfCoordinate[0]<=3.2) and (selfCoordinate[1]>=-0.03 and selfCoordinate[1]<=0):
                      flag = 0 
                      flag1= 1  
                  
              if flag1 == 1:
                  decidedMotion= self.turnMotion(origin,selfCoordinate)
                  if self.isNewMotionValid(decidedMotion):
                      boolean = self.currentlyMoving and\
                        (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                      if boolean:
                          self.interruptMotion()
                      self.clearMotionQueue()
                      if boolean:
                          self.addMotionToQueue(self.motions.standInit)
                      self.addMotionToQueue(decidedMotion)
          
                  self.startMotion()
                
      else:

        print("NO BALL DATA!!!")

  def decideMotion(self, ballCoordinate, selfCoordinate,blue_fw_l,blue_fw_r,redFw):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    
    if turningAngle > 50:
      return self.motions.turnLeft60
    elif turningAngle > 30:
      return self.motions.turnLeft40
    elif turningAngle < -50:
      return self.motions.turnRight60
    elif turningAngle < -30:
      return self.motions.turnRight40

    distanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

    if distanceFromBall < 0.22:
      return self.passBall(selfCoordinate,blue_fw_l,blue_fw_r,redFw)

    return self.motions.forwards50
    pass
    
  def returnMotion(self, ballCoordinate, selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    
    if turningAngle > 90:
      return self.motions.turnLeft180
    elif turningAngle > 50:
      return self.motions.turnLeft60
    elif turningAngle > 30:
      return self.motions.turnLeft40
    elif turningAngle < -50:
      return self.motions.turnRight60
    elif turningAngle < -30:
      return self.motions.turnRight40

    return self.motions.forwards50
    pass
    
  def turnMotion(self, ballCoordinate, selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    
    if turningAngle > 90:
      return self.motions.turnLeft180
    elif turningAngle > 50:
      return self.motions.turnLeft60
    elif turningAngle > 30:
      return self.motions.turnLeft40
    elif turningAngle < -50:
      return self.motions.turnRight60
    elif turningAngle < -30:
      return self.motions.turnRight40

    return self.motions.standInit
    pass
  
       
  def passBall(self, selfCoordinate, blue_fw_l, blue_fw_r, redFw):
      if ((redFw[0] >=(blue_fw_l[0]-0.5) and redFw[0] < blue_fw_l[0]) or (redFw[1] >= (blue_fw_l[1]-0.45) and redFw[1] < blue_fw_l[1]) or (redFw[0] <=(blue_fw_l[0]+0.5) and redFw[0] > blue_fw_l[0]) or (redFw[1] <= (blue_fw_l[1]+0.45) and redFw[1] > blue_fw_l[1])):
          return self.pass_to_right(selfCoordinate, blue_fw_r)
      else:
          return self.pass_to_left(selfCoordinate, blue_fw_l)
     
      return self.pass_to_left(selfCoordinate, blue_fw_l)
      
      
  def pass_to_right(self,selfCoordinate, rightForward):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(rightForward, selfCoordinate, robotHeadingAngle)
    if turningAngle > 90:
      return self.motions.rightSidePass
    elif turningAngle > 50:
      return self.motions.rightSidePass
    elif turningAngle > 30:
      return self.motions.rightSidePass    
    elif turningAngle <-50:
      return self.motions.leftSidePass
    elif turningAngle <-30:       
      return self.motions.leftSidePass
    else:
      return self.motions.longShoot
      
  def pass_to_left(self,selfCoordinate, leftForward):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(leftForward, selfCoordinate, robotHeadingAngle)
    if turningAngle > 90:
      return self.motions.rightSidePass
    elif turningAngle > 50:
      return self.motions.rightSidePass
    elif turningAngle > 30:
      return self.motions.rightSidePass    
    elif turningAngle <-50:
      return self.motions.leftSidePass
    elif turningAngle <-30:       
      return self.motions.leftSidePass
    else:
      return self.motions.longShoot