from commands2 import CommandBase
from constants import *
import math
from pathplannerlib import PathPlanner
from subsystems.swerve_drive import SwerveDrive
from wpilib import SmartDashboard, Timer


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

        rotationX = self.pathIndex.asWPILibState().curvature * self.pathIndex.asWPILibState().velocity

        # Use trig to get translationX and translationY
        translationMag = self.pathIndex.velocity
        translationHeading = self.pathIndex.pose.rotation().radians()
        
        translationX = math.cos(translationHeading) * translationMag
        translationY = math.sin(translationHeading) * translationMag

        # Convert to magnitudes
        rotationX /= klarryMaxRotSpeed
        translationX /= klarryMaxSpeed
        translationY /= klarryMaxSpeed

        # Clamp magnitudes
        rotationX = max(-1, min(1, rotationX))
        translationX = max(-1, min(1, translationX))
        translationY = max(-1, min(1, translationY))

        # Send to SwerveDrive
        self.drive.translateAndTurn(translationX, translationY, rotationX,
                                    applyTranslationMultiplier=False, applyRotationMultiplier=False, applySpeedModifier=False)
        
        SmartDashboard.putNumber("TranslationX", translationX)
        SmartDashboard.putNumber("TranslationY", translationY)
        SmartDashboard.putNumber("RotX", rotationX)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return (self.sampleTime >= self.path.getTotalTime())

