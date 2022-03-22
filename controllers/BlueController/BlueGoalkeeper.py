"""
Blue Team Goalkeeper robot behaviours.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import (TIME_STEP, Motions)

class Goalkeeper (SoccerRobot):
  def run(self):
    self.printSelf()
    flag = 0
    flag1 = 0
    flag2 = 0
    flag3 = 0
    fixedCoordinate = [4.22, -0.00114, 0.343]
    origin = [0,0,0]
    count_0=0
    count =0 
    count1 =0
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
        blueDef = [data[27],data[28],data[29]]
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
          if selfCoordinate[0] >= 3.56 and selfCoordinate[0] <= 4.44 and selfCoordinate[1] >= -1.47 and selfCoordinate[1] <= 1.47 and flag2==0:
              if ballCoordinate[0]>=3.4 and ballCoordinate[0]<=4.44 and ballCoordinate[1]>=-1.5 and ballCoordinate[1]<=1.5:
                  flag=1
                  decidedMotion = self.decideMotion(ballCoordinate,selfCoordinate,blue_fw_l,blue_fw_r,redFw,blueDef)
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
                  if (ballCoordinate[0]<=3.4 or ballCoordinate[0]>=4.44) and flag==1:
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
                      if (selfCoordinate[0]>=4.0 and selfCoordinate[0]<=4.5) and (selfCoordinate[1]>=-0.01 and selfCoordinate[1]<=0):
                          flag = 0 
                          flag1 =1 
                        
                  
                  elif (ballCoordinate[0]<=3.4 or ballCoordinate[0]>=4.44) and flag==0 and flag1==0:
                      decidedMotion = self.followBallDirection(ballCoordinate,selfCoordinate)
                      if self.isNewMotionValid(decidedMotion):
                          self.interruptMotion()
                          self.clearMotionQueue()
                          self.addMotionToQueue(decidedMotion)
                          self.startMotion()
                      
                  elif flag1 == 1:
                      decidedMotion= self.turnMotion(origin,selfCoordinate)
                      if decidedMotion == self.motions.standInit:
                          count = count+1
                      if count>=2:
                          flag1=0
                          count=0
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
              flag2 = 1
              if flag3==0:
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
                      if (selfCoordinate[0]>=4.0 and selfCoordinate[0]<=4.5) and (selfCoordinate[1]>=-0.01 and selfCoordinate[1]<=0):
                          flag3 = 1
              else:
                  decidedMotion= self.turnMotion(origin,selfCoordinate)
                  if decidedMotion == self.motions.standInit:
                      count1 = count1+1
                  if count1>=2:
                      flag2=0
                      flag3=0
                      count1=0
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

  # Override decideMotion
  def decideMotion(self, ballCoordinate, selfCoordinate,blue_fw_l,blue_fw_r,redFw, blueDef):
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
      if self.check_position(selfCoordinate,redFw):
          if redFw[1]>0:
            return self.motions.rightSidePass
          elif selfCoordinate[1]>redFw[1]:
            return self.motion.longShoot
          else:
            return self.motions.leftSidePass
      else:
          return self.pass_motion(selfCoordinate, blue_fw_l, blue_fw_r, redFw, blueDef)

    return self.motions.forwards50
    pass
    
  def returnMotion(self, ballCoordinate, selfCoordinate):
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

    return self.motions.forwards50
    pass
    
  def turnMotion(self, ballCoordinate, selfCoordinate):
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

    return self.motions.standInit
    pass

  def followBallDirection(self,ballCoordinate,selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    
    if turningAngle > 50:
      return self.motions.sideStepLeft
    elif turningAngle > 30:
      return self.motions.sideStepLeft
    elif turningAngle < -50:
      return self.motions.sideStepRight
    elif turningAngle < -30:
      return self.motions.sideStepRight
    else:
      return self.motions.standInit

  def check_position(self,selfCoordinate,redForward):
    
    if  redForward[0]>=3.4 and redForward[0]<=4.4 and redForward[1]>= -1.5 and redForward[1]<= 1.5:
      return True
    else:
      return False 
      
  def pass_motion(self,selfCoordinate, blue_fw_l, blue_fw_r, redFw, blueDef):
    
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(blueDef, selfCoordinate, robotHeadingAngle)
    if turningAngle <-50:
      print('defender left side pass -50')
      return self.motions.leftSidePass
    elif turningAngle <-30:       
      print('defender left side pass -30')
      return self.motions.leftSidePass
    else:
      print('in long pass')
      return self.motions.longShoot