from components.swerve_wheel import SwerveWheel
from constants import *
import math
import navx
import pathplannerlib
from wpilib import SmartDashboard
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

class SwerveDrive:
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    navX: navx.AHRS

    def __init__(self) -> None:
        self.speedMultiplier = kDefaultSpeedMultplier
        self.rotationMultiplier = kDefaultRotationMultiplier
        self.translationMultiplier = kDefaultTranslationMultiplier

        self.translationX = self.translationY = self.rotationX = 0
        self.translationMultiplier = self.rotationMultiplier = self.speedMultiplier = 1
        self.should_freeze = False

    def setup(self) -> None:
        """
        This function is automatically called after the components have been injected.
        """
        # Kinematics
        self.front_left_pose = Translation2d(0.381, 0.381)
        self.front_right_pose = Translation2d(0.381, -0.381)
        self.rear_left_pose = Translation2d(-0.381, 0.381)
        self.rear_right_pose = Translation2d(-0.381, -0.381)
        self.kinematics = SwerveDrive4Kinematics(self.front_left_pose, self.front_right_pose, self.rear_left_pose, self.rear_right_pose)

        # Odometry
        self.odometry = SwerveDrive4Odometry(self.kinematics, Rotation2d(math.radians(self.getPitch())), 
                                             (SwerveModulePosition(self.front_left.getDirectionMotorPos()), 
                                              SwerveModulePosition(self.front_right.getDirectionMotorPos()), 
                                              SwerveModulePosition(self.rear_left.getDirectionMotorPos()), 
                                              SwerveModulePosition(self.rear_right.getDirectionMotorPos())),
                                            Pose2d(x=0, y=0, angle=0))
    
    """
    CONTROL METHODS

    These essentially set up variables and info before execute is ran (like updating translationX from 0 -> 1)
    """
    def setTranslationX(self, translationX: float) -> None:
        self.translationX = translationX

    def setTranslationY(self, translationY: float) -> None:
        self.translationY = translationY
    
    def setRotationX(self, rotationX: float) -> None:
        self.rotationX = rotationX

    def freeze(self) -> None:
        self.should_freeze = True

    def unfreeze(self) -> None:
        self.should_freeze = False

    def setSpeedMultiplier(self, multiplier: float) -> None:
        self.speedMultiplier = multiplier

    def getPitch(self) -> float:
        return self.navX.getYaw()

    """
    INFO STUFF(?)
    yeah idk tbh
    """
    def updateOdometry(self) -> None:
        self.odometry.update(Rotation2d(math.radians(self.navX_sim.getDouble("Pitch").get())),
                            SwerveModulePosition(self.front_left.getDirectionMotorPos()), 
                            SwerveModulePosition(self.front_right.getDirectionMotorPos()), 
                            SwerveModulePosition(self.rear_left.getDirectionMotorPos()), 
                            SwerveModulePosition(self.rear_right.getDirectionMotorPos()))

    """
    EXECUTE
    This is ran every "tick" of the robot. This is where we update all the wheels speed and direction.
    """
    def execute(self) -> None:
        SmartDashboard.putNumber("speedMultiplier", self.speedMultiplier)

        if self.should_freeze:
            self.front_left.stopWheel()
            self.front_right.stopWheel()
            self.rear_left.stopWheel()
            self.rear_right.stopWheel()
            return

        translationX = self.translationX
        translationY = self.translationY
        rotationX = self.rotationX

        translationX *= -1
        rotationX *= -1

        # Add non-speed multipliers
        translationX *= self.translationMultiplier
        translationY *= self.translationMultiplier
        rotationX *= self.rotationMultiplier

        # Field Orientaated Drive (aka complicated math so the robot doesn't rotate while we translate or somthin idrk)
        temp = translationY * math.cos(self.navX.getYaw() * (math.pi / 180)) + translationX * math.sin(self.navX.getYaw() * (math.pi / 180))
        translationX = -translationY * math.sin(self.navX.getYaw() * (math.pi / 180)) + translationX * math.cos(self.navX.getYaw() * (math.pi / 180))
        translationY = temp

        # FOR FUTURE ROBOTICS PEOPLE: These usually would require the self.rotationX to be multiplied by (robotLength or robotWidth / 2). 
        # However, since Larry is a square, we don't use this. I'm leaving the code there just in case someone reads this and uses it.
        robotLength = 1
        robotWidth = 1

        a = translationX - rotationX #* (robotLength / 2) 
        b = translationX + rotationX #* (robotLength / 2)
        c = translationY - rotationX #* (robotWidth / 2)
        d = translationY + rotationX #* (robotWidth / 2)

        # Wheel 1 = topRight, Wheel 2 = topLeft, Wheel 3 = bottomLeft, Wheel 4 = bottomRight
        # wheel = [speed, angle]
        topRight = [math.sqrt(b ** 2 + c ** 2), math.atan2(b, c) * (180/math.pi) + 180]
        topLeft = [math.sqrt(b ** 2 + d ** 2), math.atan2(b, d) * (180/math.pi) + 180]
        bottomLeft = [math.sqrt(a ** 2 + d ** 2), math.atan2(a, d) * (180/math.pi) + 180]
        bottomRight = [math.sqrt(a ** 2 + c ** 2), math.atan2(a, c) * (180/math.pi) + 180]

        # Check if any wheels have a speed higher than 1. If so, divide all wheels by highest value
        highestSpeed = max(abs(topRight[0]), abs(topLeft[0]), abs(bottomLeft[0]), abs(bottomRight[0]))
        if highestSpeed > 1:
            topRight[0] /= highestSpeed
            topLeft[0] /= highestSpeed
            bottomLeft[0] /= highestSpeed
            bottomRight[0] /= highestSpeed
        
        # Speed modifiers
        topRight[0] *= self.speedMultiplier
        topLeft[0] *= self.speedMultiplier
        bottomLeft[0] *= self.speedMultiplier
        bottomRight[0] *= self.speedMultiplier

        # Turn wheels :D
        self.front_left.setDesiredSpeed(topLeft[0])
        self.front_left.setDesiredAngle(topLeft[1])
        
        self.front_right.setDesiredSpeed(topRight[0])
        self.front_right.setDesiredAngle(topRight[1])

        self.rear_left.setDesiredSpeed(bottomLeft[0])
        self.rear_left.setDesiredAngle(bottomLeft[1])

        self.rear_right.setDesiredSpeed(bottomRight[0])
        self.rear_right.setDesiredAngle(bottomRight[1])
