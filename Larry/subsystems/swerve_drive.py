import math

import commands2
import constants
import conversions
import ctre
import wpilib
import wpimath.controller
import navx
from subsystems.swerve_wheel import SwerveWheel


class SwerveDrive(commands2.SubsystemBase):
    """
    Our default Swerve Drive class. This contains motor definitions, 
    turning methods, moving motors from off-state, and Field Orientated
    Drive.
    """

    def __init__(self) -> None:

        super().__init__()
        # init motors
        self.leftFrontDirection = ctre.TalonFX(constants.kleftFrontDirectionID)
        self.leftFrontSpeed = ctre.TalonFX(constants.kleftFrontSpeedID)

        self.leftRearDirection = ctre.TalonFX(constants.kleftRearDirectionID)
        self.leftRearSpeed = ctre.TalonFX(constants.kleftRearSpeedID)

        self.rightFrontDirection = ctre.TalonFX(constants.krightFrontDirectionID)
        self.rightFrontSpeed = ctre.TalonFX(constants.krightFrontSpeedID)

        self.rightRearDirection = ctre.TalonFX(constants.krightRearDirectionID)
        self.rightRearSpeed = ctre.TalonFX(constants.krightRearSpeedID)

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
        self.flCANcoder = ctre.CANCoder(constants.kflCANcoderID)
        self.rlCANcoder = ctre.CANCoder(constants.krlCANcoderID)
        self.frCANcoder = ctre.CANCoder(constants.kfrCANcoderID)
        self.rrCANcoder = ctre.CANCoder(constants.krrCANcoderID)

        # init swerve modules
        self.leftFrontSwerveModule = SwerveWheel(self.leftFrontDirection, self.leftFrontSpeed, self.flCANcoder, constants.kflCANoffset, 0.0)
        self.leftRearSwerveModule = SwerveWheel(self.leftRearDirection, self.leftRearSpeed, self.rlCANcoder, constants.krlCANoffset, 0.0)

        self.rightFrontSwerveModule = SwerveWheel(self.rightFrontDirection, self.rightFrontSpeed, self.frCANcoder, constants.kfrCANoffset, 0.0)
        self.rightRearSwerveModule = SwerveWheel(self.rightRearDirection, self.rightRearSpeed, self.rrCANcoder, constants.krrCANoffset, 0.0)

        self.wheels = [self.leftFrontSwerveModule, self.rightFrontSwerveModule, self.leftRearSwerveModule, self.rightRearSwerveModule]

        self.navX = navx.AHRS.create_spi()
        
        self.PDP = wpilib.PowerDistribution(0, wpilib.PowerDistribution.ModuleType.kCTRE)

        self.pidController = wpimath.controller.PIDController(constants.kChargeP, constants.kChargeI, constants.kChargeD)
        self.onChargeStation = False

        self.inTankMode = False
        self.inSwerveMode = True

    def turnWheel(self, module: SwerveWheel, direction: float, magnitude: float) -> None:
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

        if module.turnToOptimizedAngle(direction):
            magnitude *= -1

        if magnitude == 0:
            return
        
        module.move(magnitude)

    def translate(self, direction: float, magnitude: float):
        """
        Allows for moving up, down, left, and right without rotating 
        the robot.

        :param direction: The angle the robot should travel at 
        (ranging from 0 to 360 degrees).
        :param magnitude: The magnitude the robot should travel at 
        (ranging from 0 to 1, 1 being 100% power)
        """

        self.turnWheel(self.leftFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.leftRearSwerveModule, direction, magnitude)
        self.turnWheel(self.rightFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.rightRearSwerveModule, direction, magnitude)

    def translateAndTurn(self, translationX: float, translationY: float, rotX: float) -> None:
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
        translationX *= -1
        rotX *= -1

        # Field Orientaated Drive (aka complicated math so the robot doesn't rotate while we translate or somthin idrk)
        temp = translationY * math.cos(self.getYaw() * (math.pi / 180)) + translationX * math.sin(self.getYaw() * (math.pi / 180))
        translationX = -translationY * math.sin(self.getYaw() * (math.pi / 180)) + translationX * math.cos(self.getYaw() * (math.pi / 180))
        translationY = temp

        # FOR FUTURE ROBOTICS PEOPLE: These usually would require the rotX to be multiplied by (robotLength or robotWidth / 2). 
        # However, since Larry is a square, we don't use this. I'm leaving the code there just in case someone reads this and uses it.
        robotLength = 1
        robotWidth = 1

        a = translationX - rotX #* (robotLength / 2) 
        b = translationX + rotX #* (robotLength / 2)
        c = translationY - rotX #* (robotWidth / 2)
        d = translationY + rotX #* (robotWidth / 2)

        if constants.kDebug:
            wpilib.SmartDashboard.putString("ABCD", str([a, b, c, d]))

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

        wpilib.SmartDashboard.putNumber("topRightAngle", topRight[1])
        wpilib.SmartDashboard.putNumber("topLeftAngle", topLeft[1])
        wpilib.SmartDashboard.putNumber("bottomLeftAngle", bottomLeft[1])
        wpilib.SmartDashboard.putNumber("bottomRightAngle", bottomRight[1])
        wpilib.SmartDashboard.putNumber("topRightRealAngle", self.rightFrontSwerveModule.getCurrentAngle() % 360)
        wpilib.SmartDashboard.putNumber("topLeftRealAngle", self.leftFrontSwerveModule.getCurrentAngle() % 360)
        wpilib.SmartDashboard.putNumber("bottomLeftRealAngle", self.leftRearSwerveModule.getCurrentAngle() % 360)
        wpilib.SmartDashboard.putNumber("bottomRightRealAngle", self.rightRearSwerveModule.getCurrentAngle() % 360)
        

        # Stops robot from moving while no controller values are being returned, but allow robot to still be able to turn wheels
        if translationX == 0 and translationY == 0 and rotX == 0:
            self.stopAllMotors()
            return

        # Turn wheels :D
        self.turnWheel(self.leftFrontSwerveModule, topLeft[1], topLeft[0])
        self.turnWheel(self.rightFrontSwerveModule, topRight[1], topRight[0])
        self.turnWheel(self.leftRearSwerveModule, bottomLeft[1], bottomLeft[0])
        self.turnWheel(self.rightRearSwerveModule, bottomRight[1], bottomRight[0])

    def turnInPlace(self, turnPower: float):
        """
        Sets all motors to pre-defined angles to allow for optimized 
        turning back and forth.

        :param turnPower: The magnitude of the turning (ranging from 
        -1 to 1, 1 being 100% power forward)
        """
        self.turnWheel(self.leftFrontSwerveModule, 45.0, turnPower)
        self.turnWheel(self.rightFrontSwerveModule, 135.0, turnPower)
        self.turnWheel(self.rightRearSwerveModule, 225.0, turnPower)
        self.turnWheel(self.leftRearSwerveModule, 315.0, turnPower)

    def stopAllMotors(self):
        """
        Stops all motors.
        """
        self.leftFrontSwerveModule.stopAllMotors()
        self.leftRearSwerveModule.stopAllMotors()
        self.rightFrontSwerveModule.stopAllMotors()
        self.rightRearSwerveModule.stopAllMotors()

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
        self.turnWheel(self.leftFrontSwerveModule, 0.0, 0.0)
        self.turnWheel(self.leftRearSwerveModule, 0.0, 0.0)
        self.turnWheel(self.rightFrontSwerveModule, 0.0, 0.0)
        self.turnWheel(self.rightRearSwerveModule, 0.0, 0.0)
        self.stopAllMotors()

    def getPosFromOffState(self):
        """
        Moves all motors to the correct position on startup.
        """
        self.leftFrontSwerveModule.CANtoTalon()
        self.leftRearSwerveModule.CANtoTalon()
        self.rightFrontSwerveModule.CANtoTalon()
        self.rightRearSwerveModule.CANtoTalon()

    def reset(self):
        """
        Resets the navX and sets all motor positions to 0.
        """
        self.navX.reset()

        self.leftFrontDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.leftFrontSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.leftRearDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.leftRearSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightFrontDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightFrontSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightRearDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightRearSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)

    def enableTankDrive(self) -> bool:
        """
        Enables tank drive mode. This aligns all wheels into a tank 
        drive formation, disabling turning and moving simultaneously.

        The main use for this is to run automation code that was
        intended for tank drive drivetrains.

        :returns: True if successfully enabled, False if otherwise.
        """
        self.flushWheels()
        for wheel in self.wheels:
            if wheel.getCurrentAngle() != 0.0:
                return False

        self.inTankMode = True
        self.inSwerveMode = False
        return True
    
    def isInTankDrive(self) -> bool:
        """
        Used for getting the current drive mode of the robot.

        :returns: True if in tank drive mode, False if otherwise.
        """
        return self.inTankMode
    
    def enableSwerveDrive(self) -> bool:
        """
        Enables swerve drive mode (enabled by default) and disables 
        tank drive mode. This allows for turning and moving 
        simultaneously.

        This is used mainly for teleop purposes to allow for more 
        control over movement.

        :returns: True if successfully enabled, False if otherwise.
        """
        try:
            self.flushWheels()
            self.inTankMode = False
            self.inSwerveMode = True
            return True
        except:
            return False # The chance of this actually failing is literally only if like a motor gets unplugged lmao
        
    def isInSwerveDrive(self) -> bool:
        """
        Used for getting the current drive mode of the robot.

        :returns: True if in swerve drive mode, False if otherwise.
        """
        return self.inSwerveMode
