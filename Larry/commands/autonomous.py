from math import atan2, cos, degrees, pi, sin

from commands2 import CommandBase
from constants import *
from pathplannerlib import PathPlanner
from subsystems.swerve_drive import SwerveDrive
from wpilib import RobotBase, SmartDashboard, Timer


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

        self.desiredAngle = 0
        self.desiredPosX = self.path.getInitialPose().X()
        self.desiredPosY = self.path.getInitialPose().Y()
        self.drive.navX.reset()

    def execute(self) -> None:
        self.sampleTime = Timer.getFPGATimestamp() - self.startTime
        self.pathIndex = self.path.sample(self.sampleTime)

        # Use trig to get translationX and translationY
        translationHeading = self.pathIndex.pose.rotation().radians()
        translationX = sin(translationHeading) * -self.pathIndex.velocity
        translationY = cos(translationHeading) * -self.pathIndex.velocity

        rotationX = self.pathIndex.holonomicAngularVelocity

        self.desiredAngle += degrees(rotationX) / 50 # gets degrees per tick
        self.desiredPosX = self.pathIndex.pose.X()
        self.desiredPosY = self.pathIndex.pose.Y()

        # Fix drift
        """
        if RobotBase.isReal():
            angleDrift = self.desiredAngle - self.drive.getYaw()
            print(angleDrift)
            rotationX -= 1 / ((180 / krotationMagnitudeMM)**3) * (angleDrift ** 3) # This increases the rotationX depending on how far off we are currently

        """
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

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        return self.sampleTime >= self.path.getTotalTime()
    
class MoveMeters(CommandBase):
    """
    Moves the robot in a straight line the chosen amount of meters.
    """
    def __init__(self, drive: SwerveDrive, distanceX: float, distanceY: float) -> None:
        super().__init__()

        self.drive = drive
        self.addRequirements([self.drive])

        self.distanceX = distanceX
        self.distanceY = distanceY

    def initialize(self) -> None:
        pose = self.drive.getPose()
        self.finalDispX = self.distanceX - pose.X() # Adjacent angle
        self.finalDispY = self.distanceY - pose.Y() # Opposite angle
        self.finalX = self.distanceX + pose.X()
        self.finalY = self.distanceY + pose.Y()

        # Trajectory angle
        self.trajAngle = atan2(self.finalDispX, self.finalDispY) * (180 / pi)
        if self.trajAngle < 0:
            self.trajAngle += 360

        self.drive.pointWheelsAtAngle(self.trajAngle)
        self.magnitude = 0

    def execute(self) -> None:
        if not self.drive.areWheelsAtCorrectAngle():
            self.drive.stopAllMotors()
            self.drive.pointWheelsAtAngle(self.trajAngle)
            return
        
        self.magnitude += kdefaultMagIncrease
        self.magnitude = min(kmagMax, max(-kmagMax, self.magnitude))
        self.drive.moveAtConstantMagnitude(self.magnitude)

    def end(self, interrupted: bool) -> None:
        self.drive.stopAllMotors()

    def isFinished(self) -> bool:
        pose = self.drive.getPose()
        return pose.X() == self.finalX and pose.Y() == self.finalY
    
class FollowPathNoRotation(CommandBase):
    """
    Follows the specified PathPlanner path without rotation.

    Due to current development deadline limitations, following a path from PathPlanner with rotation is currently unavailable.
    """

    def __init__(self, swerveDrive: SwerveDrive, pathName: str, 
                 pathConstraints: PathConstraints=kdefaultPathConstraints, isReversed: bool = False) -> None:
        super().__init__()

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True
    
class AutoRotate(CommandBase):
    """
    Rotates the robot to the specified degree related to the NavX sensor.
    """

    def __init__(self) -> None:
        super().__init__()

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return True

