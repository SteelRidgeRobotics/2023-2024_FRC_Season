from commands2 import SubsystemBase
from constants import *
import ctre
from ctre.sensors import CANCoder
from math import *
import navx
from subsystems.swerve_wheel import SwerveWheel
from wpilib import PowerDistribution, RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry


class SwerveDrive(SubsystemBase):
    """
    Our default Swerve Drive class. This contains motor definitions, 
    turning methods, moving motors from off-state, and Field Orientated
    Drive.
    """

    def __init__(self) -> None:

        super().__init__()
        # init motors
        self.leftFrontDirection = ctre.TalonFX(kleftFrontDirectionID)
        self.leftFrontSpeed = ctre.TalonFX(kleftFrontSpeedID)

        self.leftRearDirection = ctre.TalonFX(kleftRearDirectionID)
        self.leftRearSpeed = ctre.TalonFX(kleftRearSpeedID)

        self.rightFrontDirection = ctre.TalonFX(krightFrontDirectionID)
        self.rightFrontSpeed = ctre.TalonFX(krightFrontSpeedID)

        self.rightRearDirection = ctre.TalonFX(krightRearDirectionID)
        self.rightRearSpeed = ctre.TalonFX(krightRearSpeedID)

        # fix inverse
        self.leftFrontSpeed.setInverted(True)
        self.leftRearSpeed.setInverted(True)

        self.rightFrontSpeed.setInverted(True)
        self.rightRearSpeed.setInverted(True)

        self.leftFrontDirection.setInverted(False)
        self.leftRearDirection.setInverted(False)

        self.rightFrontDirection.setInverted(False)
        self.rightRearDirection.setInverted(False)

        # init CAN coders
        self.flCANcoder = CANCoder(kflCANcoderID)
        self.rlCANcoder = CANCoder(krlCANcoderID)
        self.frCANcoder = CANCoder(kfrCANcoderID)
        self.rrCANcoder = CANCoder(krrCANcoderID)

        # init swerve modules
        self.leftFrontWheel = SwerveWheel(self.leftFrontDirection, self.leftFrontSpeed, self.flCANcoder, kflCANoffset, 0.0)
        self.leftRearWheel = SwerveWheel(self.leftRearDirection, self.leftRearSpeed, self.rlCANcoder, krlCANoffset, 0.0)
        self.rightFrontWheel = SwerveWheel(self.rightFrontDirection, self.rightFrontSpeed, self.frCANcoder, kfrCANoffset, 0.0)
        self.rightRearWheel = SwerveWheel(self.rightRearDirection, self.rightRearSpeed, self.rrCANcoder, krrCANoffset, 0.0)

        self.navX = navx.AHRS.create_spi()
        self.angleOffset = 0

        # module positions
        self.leftFrontPosition = Translation2d(krobotSize, krobotSize)
        self.rightFrontPosition = Translation2d(krobotSize, -krobotSize)
        self.leftRearPosition = Translation2d(-krobotSize, krobotSize)
        self.rightRearPosition = Translation2d(-krobotSize, -krobotSize)

        self.kinematics = SwerveDrive4Kinematics(self.leftFrontPosition, self.rightFrontPosition, self.leftRearPosition, self.rightRearPosition)

        self.odometry = SwerveDrive4Odometry(self.kinematics, Rotation2d.fromDegrees(self.getYaw()),
                                             (self.leftFrontWheel.getPosition(), self.rightFrontWheel.getPosition(),
                                              self.leftRearWheel.getPosition(), self.rightRearWheel.getPosition()))
        
        self.PDP = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE)

        # Movement modifiers
        self.speedMultiplier = kDefaultSpeedMultplier
        self.rotationMultiplier = kDefaultRotationMultiplier
        self.translationMultiplier = kDefaultTranslationMultiplier

    def turnWheel(self, module: SwerveWheel, direction: float, magnitude: float, 
                  applySpeedMultiplier: bool=False) -> None:
        """
        Turns a swerve wheel based on the provided direction and 
        magnitude.

        :param module: The swerve wheel to turn.
        :param direction: The angle the wheel should turn to 
        (ranging from 0 to 360 degrees).
        :param magnitude: The magnitude the wheel should turn at 
        (ranging from 0 to 1, 1 being 100% power)
        """

        # Magnitude clamp between -1 and 1
        magnitude = max(-1.0, min(1.0, magnitude))
        if applySpeedMultiplier:
            magnitude *= self.speedMultiplier

        module.turnToOptimizedAngle(direction)

        if magnitude == 0:
            return
        
        module.move(magnitude)

    def translate(self, direction: float, magnitude: float, 
                  applyTranslationMultiplier: bool=True, applySpeedMultiplier: bool=True):
        """
        Allows for moving up, down, left, and right without rotating 
        the robot.

        :param direction: The angle the robot should travel at 
        (ranging from 0 to 360 degrees).
        :param magnitude: The magnitude the robot should travel at 
        (ranging from 0 to 1, 1 being 100% power)
        """
        if applyTranslationMultiplier:
            magnitude *= self.translationMultiplier
        if applySpeedMultiplier:
            magnitude *= self.speedMultiplier

        self.turnWheel(self.leftFrontWheel, direction, magnitude)
        self.turnWheel(self.leftRearWheel, direction, magnitude)
        self.turnWheel(self.rightFrontWheel, direction, magnitude)
        self.turnWheel(self.rightRearWheel, direction, magnitude)

    def translateAndTurn(self, translationX: float, translationY: float, rotX: float, 
                         applyTranslationMultiplier: bool=True, applyRotationMultiplier: bool=True, applySpeedModifier: bool=True) -> None:
        """
        This is the default movement method for swerve drive.

        This allows for turning and moving simultaneously and 
        includes further movement optimizations.

        :param translationX: The magnitude of moving left and right 
        (ranging from -1 to 1)
        :param translationY: The magnitude of moving up and down 
        (ranging from -1 to 1)
        :param rotX: The magnitude of rotating left and right 
        (ranging from -1 to 1)
        """
        SmartDashboard.putNumber("translationX", translationX)
        SmartDashboard.putNumber("translationY", translationY)
        SmartDashboard.putNumber("rotationX", rotX)

        translationX *= -1
        rotX *= -1

        # Add non-speed multipliers
        if applyTranslationMultiplier:
            translationX *= self.translationMultiplier
            translationY *= self.translationMultiplier
        if applyRotationMultiplier:
            rotX *= self.rotationMultiplier

        # Field Orientated Drive (aka complicated math so the robot doesn't rotate while we translate or somthin idrk)
        temp = translationY * cos((self.getYaw() + self.angleOffset) * (pi / 180)) + translationX * sin((self.getYaw() + self.angleOffset) * (pi / 180))
        translationX = -translationY * sin((self.getYaw() + self.angleOffset) * (pi / 180)) + translationX * cos((self.getYaw() + self.angleOffset) * (pi / 180))
        translationY = temp

        # FOR FUTURE ROBOTICS PEOPLE: These usually would require the rotX to be multiplied by (robotLength or robotWidth / 2). 
        # However, since Larry is a square, we don't use this. I'm leaving the code there just in case someone reads this and uses it.
        robotLength = 1
        robotWidth = 1

        a = translationX - rotX #* (robotLength / 2) 
        b = translationX + rotX #* (robotLength / 2)
        c = translationY - rotX #* (robotWidth / 2)
        d = translationY + rotX #* (robotWidth / 2)

        # Wheel 1 = topRight, Wheel 2 = topLeft, Wheel 3 = bottomLeft, Wheel 4 = bottomRight
        # wheel = [speed, angle]
        topRight = [sqrt(b ** 2 + c ** 2), atan2(b, c) * (180/pi) + 180]
        topLeft = [sqrt(b ** 2 + d ** 2), atan2(b, d) * (180/pi) + 180]
        bottomLeft = [sqrt(a ** 2 + d ** 2), atan2(a, d) * (180/pi) + 180]
        bottomRight = [sqrt(a ** 2 + c ** 2), atan2(a, c) * (180/pi) + 180]

        if kDebug:
            SmartDashboard.putNumberArray("topRight", topRight)
            SmartDashboard.putNumberArray("bottomRight", bottomRight)
            SmartDashboard.putNumberArray("topLeft", topLeft)
            SmartDashboard.putNumberArray("bottomLeft", bottomLeft)

        # Check if any wheels have a speed higher than 1. If so, divide all wheels by highest value
        highestSpeed = max(abs(topRight[0]), abs(topLeft[0]), abs(bottomLeft[0]), abs(bottomRight[0]))
        if highestSpeed > 1:
            topRight[0] /= highestSpeed
            topLeft[0] /= highestSpeed
            bottomLeft[0] /= highestSpeed
            bottomRight[0] /= highestSpeed

        if RobotBase.isReal() and kDebug:
            SmartDashboard.putNumber("topRightRealAngle", self.rightFrontWheel.getCurrentAngle() % 360)
            SmartDashboard.putNumber("topLeftRealAngle", self.leftFrontWheel.getCurrentAngle() % 360)
            SmartDashboard.putNumber("bottomLeftRealAngle", self.leftRearWheel.getCurrentAngle() % 360)
            SmartDashboard.putNumber("bottomRightRealAngle", self.rightRearWheel.getCurrentAngle() % 360)
        
        # Stops robot from moving while no controller values are being returned
        if translationX == 0 and translationY == 0 and rotX == 0:
            self.stopAllMotors()
            return
        
        # Speed modifiers
        if applySpeedModifier:
            topRight[0] *= self.speedMultiplier
            topLeft[0] *= self.speedMultiplier
            bottomLeft[0] *= self.speedMultiplier
            bottomRight[0] *= self.speedMultiplier

        # Turn wheels :D
        self.turnWheel(self.leftFrontWheel, topLeft[1], topLeft[0])
        self.turnWheel(self.rightFrontWheel, topRight[1], topRight[0])
        self.turnWheel(self.leftRearWheel, bottomLeft[1], bottomLeft[0])
        self.turnWheel(self.rightRearWheel, bottomRight[1], bottomRight[0])

    def turnInPlace(self, turnPower: float, applyRotationMultiplier: bool=True, applySpeedMultiplier: bool=True) -> None:
        """
        Sets all motors to pre-defined angles to allow for optimized 
        turning back and forth.

        :param turnPower: The magnitude of the turning (ranging from 
        -1 to 1, 1 being 100% power forward)
        """
        if applyRotationMultiplier:
            turnPower *= self.rotationMultiplier
        if applySpeedMultiplier:
            turnPower *= self.speedMultiplier

        self.turnWheel(self.leftFrontWheel, 45.0, turnPower)
        self.turnWheel(self.rightFrontWheel, 135.0, turnPower)
        self.turnWheel(self.rightRearWheel, 225.0, turnPower)
        self.turnWheel(self.leftRearWheel, 315.0, turnPower)

    def stopAllMotors(self):
        """
        Stops all motors.
        """
        self.leftFrontWheel.stopAllMotors()
        self.leftRearWheel.stopAllMotors()
        self.rightFrontWheel.stopAllMotors()
        self.rightRearWheel.stopAllMotors()

    def getYaw(self):
        """
        Used for geting the yaw of the NavX.

        :returns: the yaw of the NavX (from -180 to 180)
        """
        return self.navX.getYaw()
        
    def getPitch(self):
        """
        Used for geting the pitch of the NavX.

        :returns: the pitch of the NavX (from -180 to 180)
        """
        return self.navX.getPitch()
    
    def flushWheels(self):
        """
        Sets all swerve wheels to angle 0 with 0 magnitude, then stops.
        """
        self.turnWheel(self.leftFrontWheel, 0.0, 0.0)
        self.turnWheel(self.leftRearWheel, 0.0, 0.0)
        self.turnWheel(self.rightFrontWheel, 0.0, 0.0)
        self.turnWheel(self.rightRearWheel, 0.0, 0.0)
        self.stopAllMotors()

    def getPosFromOffState(self):
        """
        Moves all motors to the correct position on startup.
        """
        self.leftFrontWheel.CANtoTalon()
        self.leftRearWheel.CANtoTalon()
        self.rightFrontWheel.CANtoTalon()
        self.rightRearWheel.CANtoTalon()

    def reset(self):
        """
        Resets the navX and sets all motor positions to 0.
        """
        self.navX.reset()

        self.leftFrontDirection.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.leftFrontSpeed.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.leftRearDirection.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.leftRearSpeed.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.rightFrontDirection.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.rightFrontSpeed.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.rightRearDirection.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.rightRearSpeed.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

    """
    DRIVING MULTIPLIERS
    """
    def setSpeedMultiplier(self, newSpeed: float) -> None:
        """
        Sets the new speed multiplier.

        The speed multiplier is factored into all turnWheel functions.

        Values between 0-1 are accepted, 0 being 0% power and 1 being maximum power.
        """
        self.speedMultiplier = newSpeed

    def getSpeedMultiplier(self) -> float:
        """
        Gets the speed multiplier.

        The speed multiplier is factored into all turnWheel functions.
        """
        return self.speedMultiplier
    
    def getDefaultSpeedMultiplier(self) -> float:
        """
        Gets the default speed multiplier.

        The speed multiplier is factored into all turnWheel functions.
        """
        return self.defaultSpeedMultiplier
    
    def setDefaultSpeedMultiplier(self, newDefSpeed: float) -> float:
        """
        Sets the default speed multiplier.

        The speed multiplier is factored into all turnWheel functions.
        """
        self.defaultSpeedMultiplier = newDefSpeed
    
    def setRotationMultiplier(self, newSpeed: float) -> None:
        """
        Sets the new rotation multiplier.

        The rotation multiplier is factored into all rotation-related calculations.

        Values between 0-1 are accepted, 0 being 0% power and 1 being maximum power.
        """
        self.rotationMultiplier = newSpeed

    def getRotationMultiplier(self) -> float:
        """
        Gets the rotation multiplier.

        The rotation multiplier is factored into all rotation-related calculations.
        """
        return self.rotationMultiplier
    
    def getDefaultRotationMultiplier(self) -> float:
        """
        Gets the default rotation multiplier.

        The rotation multiplier is factored into all rotation-related calculations.
        """
        return self.defaultRotationMultiplier
    
    def setDefaultRotationMultiplier(self, newDefSpeed: float) -> None:
        """
        Sets the default rotation multiplier.

        The rotation multiplier is factored into all rotation-related calculations.

        Values between 0-1 are accepted, 0 being 0% power and 1 being maximum power.
        """
        self.defaultRotationMultiplier = newDefSpeed
    
    def setTranslationMultiplier(self, newSpeed: float) -> None:
        """
        Sets the new translation multiplier.

        The translation multiplier is factored into all translation-related calculations.
        """
        self.translationMultiplier = newSpeed

    def getTranslationMultiplier(self) -> float:
        """
        Gets the translation multiplier.

        The translation multiplier is factored into all translation-related calculations.
        """
        return self.translationMultiplier
    
    def getDefaultTranslationMultiplier(self) -> float:
        """
        Gets the default translation multiplier.

        The translation multiplier is factored into all translation-related calculations.
        """
        return self.defaultTranslationMultiplier
    
    def setDefaultTranslationMultiplier(self, newDefSpeed: float) -> None:
        """
        Sets the default translation multiplier.

        The translation multiplier is factored into all translation-related calculations.

        Values between 0-1 are accepted, 0 being 0% power and 1 being maximum power.
        """
        self.defaultTranslationMultiplier = newDefSpeed

    def addToAngleOffset(self, num: float) -> None:
        """
        Adds the given number to the current angle offset.

        Angle offset is used when calculating FOD while moving. This provides an easy way to correct incorrect starting gyro angles.

        Angle offset is capped between -180 and 180.
        """
        self.angleOffset =  max(min(self.angleOffset + num, 180), -180)
        SmartDashboard.putNumber("Angle Offset", self.angleOffset)

    def getOdometry(self) -> SwerveDrive4Odometry:
        return self.odometry
    
    def updateOdometry(self) -> None:
        gyroAngle = Rotation2d.fromDegrees(self.getYaw())

        self.odometry.update(gyroAngle, self.leftFrontWheel.getPosition(), self.rightFrontWheel.getPosition(), self.leftRearWheel.getPosition(), self.rightRearWheel.getPosition())

    def updateWheelPositions(self) -> None:
        self.leftFrontWheel.updatePostion()
        self.rightFrontWheel.updatePostion()
        self.leftRearWheel.updatePostion()
        self.rightRearWheel.updatePostion()
