from commands2 import CommandBase
from constants import *
import math
from pathplannerlib import PathPlanner
from subsystems.swerve_drive import SwerveDrive
from wpilib import Timer


class FollowPath(CommandBase):
    """
    Follows the given path planner made and saved in pathplannerlib.
    """
    def __init__(self, swerveDrive: SwerveDrive, pathName: str, 
                 pathConstraints: PathConstraints=kdefaultPathConstraints, isReversed: bool = False) -> None:
        super().__init__()

        self.drive = swerveDrive
        self.pathConstraints = pathConstraints
        self.path = PathPlanner.loadPath(pathName, pathConstraints, isReversed)

        self.addRequirements([self.drive])
        
    def initialize(self) -> None:
        self.startTime = Timer.getFPGATimestamp()

    def execute(self) -> None:
        currentTime = Timer.getFPGATimestamp()

        self.sampleTime = currentTime - self.startTime

        self.pathIndex = self.path.sample(self.sampleTime)

         # WARNING: THIS WILL NOT WORK!!! We have to get the speed of each wheel on the robot and convert this value into a value from -1 to 1 magnitude
         # Calculates angleVel
        angleVel = self.pathIndex.asWPILibState().curvature * self.pathIndex.asWPILibState().velocity

        # Sets angle value to magnitude for larry
        # (put something here :p)

        print("{Time: " + str(round(self.sampleTime, 2)) + ", angleVel: " + str(round(angleVel, 3)) + "}")

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return (self.sampleTime >= self.path.getTotalTime())

