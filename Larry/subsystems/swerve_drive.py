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

        self.navX = navx.AHRS.create_spi()
        
        self.PDP = wpilib.PowerDistribution(0, wpilib.PowerDistribution.ModuleType.kCTRE)

        self.pidController = wpimath.controller.PIDController(constants.kChargeP, constants.kChargeI, constants.kChargeD)
        self.onChargeStation = False

    def turnWheel(self, module: SwerveWheel, direction: float, magnitude: float):
        self.units = conversions.convertDegreesToTalonFXUnits(direction)

        if magnitude >= 1.0:
            magnitude = 1.0
        elif magnitude <= -1.0:
            magnitude = -1.0

        # find current angle
        currentAngle = conversions.convertTalonFXUnitsToDegrees(module.directionMotor.getSelectedSensorPosition()/constants.ksteeringGearRatio)
        """
        # see if the abs value is greater than 180
        if math.fabs(direction) >= 180.0:
            # find the abs value of the opposite angle
            opposAngle = math.fabs(direction) - 180.0
        else:
            # find the abs value of the opposite angle
            opposAngle = math.fabs(direction) + 180.0
        """
        
        if direction < 0:
            opposAngle = direction + 180
            negAngle = 360 + direction
        elif direction > 0:
            opposAngle = direction - 180
            negAngle = direction - 360
            
        else:
            if conversions.sign(direction) == -1:
                opposAngle = -180
                negAngle = 0
            else:
                opposAngle = 180
                negAngle = 0

        # print some stats for debugging
        wpilib.SmartDashboard.putNumber(" Abs Opposite Angle -", opposAngle)
        wpilib.SmartDashboard.putNumber(" Neg Angle -", negAngle)
        # check if the joystick is in use
        if magnitude != 0.0:
            """
            # this is to test that if 360 or zero is closer it goes to 0
            if (direction == 0.0 or direction == 180.0) and math.fabs(360 - currentAngle) <= math.fabs(
                    currentAngle - opposAngle):
                # this means that 360 or zero is the shortest distance
                # now we have to find if 0.0 is the direction or the opposite angle
                if direction == 0.0:
                    module.turn(self.units * constants.ksteeringGearRatio)
                    module.move(magnitude)
                else:
                    module.turn(conversions.convertDegreesToTalonFXUnits(opposAngle) * constants.ksteeringGearRatio)
                    module.move(-magnitude)
            """
        
            """
            # if negAngle is closer
            if math.fabs(currentAngle - direction) >= math.fabs(currentAngle - negAngle):
                module.turn(conversions.convertDegreesToTalonFXUnits(negAngle + rev) * constants.ksteeringGearRatio)
                wpilib.SmartDashboard.putNumber("1", math.fabs(currentAngle - direction))
                wpilib.SmartDashboard.putNumber("-", math.fabs(currentAngle - negAngle))
                wpilib.SmartDashboard.putBoolean("Using - ANGLE: ", True)
            # if the original angle is closer   
            elif math.fabs(currentAngle - direction) <= math.fabs(currentAngle - opposAngle):
                # turn to the original angle
                module.turn(conversions.convertDegreesToTalonFXUnits(direction + rev) * constants.ksteeringGearRatio)
                # move in the normal way
                module.move(magnitude)
                wpilib.SmartDashboard.putBoolean("Using - ANGLE: ", False)
            else:  # the opposite angle is closer
                # turn to the other angle
                module.turn(conversions.convertDegreesToTalonFXUnits(opposAngle + rev) * constants.ksteeringGearRatio)
                # move in the opposite direction
                module.move(-magnitude)
                wpilib.SmartDashboard.putBoolean("Using - ANGLE: ", False)
            """
            module.turn(constants.ksteeringGearRatio * conversions.convertDegreesToTalonFXUnits(conversions.getclosest(currentAngle, direction, magnitude)[0]))
            module.move(conversions.getclosest(currentAngle, direction, magnitude)[1])

            wpilib.SmartDashboard.putNumber(" Wanted Angle -", direction)
            wpilib.SmartDashboard.putNumber("Given Angle", conversions.getclosest(currentAngle, direction, magnitude)[0])
            wpilib.SmartDashboard.putNumber("Current Angle", currentAngle)

    def translate(self, direction: float, magnitude: float):
        self.turnWheel(self.leftFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.leftRearSwerveModule, direction, magnitude)
        self.turnWheel(self.rightFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.rightRearSwerveModule, direction, magnitude)

    def translateAndTurn(self, translationX: float, translationY: float, rotX: float):
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
        roundingAm = 3

        topRight = [math.sqrt(b ** 2 + c ** 2), math.atan2(b, c) * (180/math.pi) + 180]
        topLeft = [math.sqrt(b ** 2 + d ** 2), math.atan2(b, d) * (180/math.pi) + 180]
        bottomLeft = [math.sqrt(a ** 2 + d ** 2), math.atan2(a, d) * (180/math.pi) + 180]
        bottomRight = [math.sqrt(a ** 2 + c ** 2), math.atan2(a, c) * (180/math.pi) + 180]

        # Check if any wheels have a speed higher than 1. If so, divide all wheels by highest value
        highestSpeed = max(topRight[0], topLeft[0], bottomLeft[0], bottomRight[0])
        if highestSpeed > 1:
            topRight[0] = topRight[0] / highestSpeed
            topLeft[0] = topLeft[0] / highestSpeed
            bottomLeft[0] = bottomLeft[0] / highestSpeed
            bottomRight[0] = bottomRight[0] / highestSpeed

        wpilib.SmartDashboard.putString("topRight", str(topRight))
        wpilib.SmartDashboard.putString("topLeft", str(topLeft))
        wpilib.SmartDashboard.putString("bottomLeft", str(bottomLeft))
        wpilib.SmartDashboard.putString("bottomRight", str(bottomRight))

        self.turnWheel(self.leftFrontSwerveModule, topLeft[1], topLeft[0])
        self.turnWheel(self.rightFrontSwerveModule, topRight[1], topRight[0])
        self.turnWheel(self.leftRearSwerveModule, bottomLeft[1], bottomLeft[0])
        self.turnWheel(self.rightRearSwerveModule, bottomRight[1], bottomRight[0])

    def turnInPlace(self, turnPower: float):
        self.turnWheel(self.leftFrontSwerveModule, 45.0, turnPower)
        self.turnWheel(self.rightFrontSwerveModule, 135.0, turnPower)
        self.turnWheel(self.rightRearSwerveModule, 225.0, turnPower)
        self.turnWheel(self.leftRearSwerveModule, 315.0, turnPower)

    def stopAllMotors(self):
        self.leftFrontSwerveModule.stopAllMotors()
        self.leftRearSwerveModule.stopAllMotors()
        self.rightFrontSwerveModule.stopAllMotors()
        self.rightRearSwerveModule.stopAllMotors()

    #    wpilib.SmartDashboard.putNumberArray("Code Offsets ", [self.leftFrontSwerveModule.offset, self.rightFrontSwerveModule.offset, self.leftRearSwerveModule.offset, self.rightRearSwerveModule.offset])


    def getYaw(self):

        return self.navX.getYaw()
        
    def getPitch(self):

        return self.navX.getPitch()

    def flushWheels(self):
        self.turnWheel(self.leftFrontSwerveModule, 0.0, 0.01)
        self.turnWheel(self.leftRearSwerveModule, 0.0, 0.01)
        self.turnWheel(self.rightFrontSwerveModule, 0.0, 0.01)
        self.turnWheel(self.rightRearSwerveModule, 0.0, 0.01)

        self.stopAllMotors()

    def getPosFromOffState(self):

        self.leftFrontSwerveModule.CANtoTalon()
        self.leftRearSwerveModule.CANtoTalon()
        self.rightFrontSwerveModule.CANtoTalon()
        self.rightRearSwerveModule.CANtoTalon()

    def moveWhileSpinning(self, leftx: float, lefty: float, turnPower: float):
        straff = -lefty * math.sin(self.getYaw()) + leftx * math.cos(self.getYaw())
        fwrd = lefty * math.cos(self.getYaw()) + leftx * math.sin(self.getYaw())
        a = straff - turnPower * (constants.klength / constants.kr)
        b = straff + turnPower * (constants.klength / constants.kr)
        c = fwrd - turnPower * (constants.kwidth / constants.kr)
        d = fwrd + turnPower * (constants.kwidth / constants.kr)

        frspeed = math.sqrt(b ** 2 + c ** 2)
        flspeed = math.sqrt(b ** 2 + d ** 2)
        rlspeed = math.sqrt(a ** 2 + d ** 2)
        rrspeed = math.sqrt(a ** 2 + c ** 2)

        frangle = math.atan2(b, c) * (180 / math.pi)
        flangle = math.atan2(b, d) * (180 / math.pi)
        rlangle = math.atan2(a, d) * (180 / math.pi)
        rrangle = math.atan2(a, c) * (180 / math.pi)

        # the block below checks for the highest speed that a wheel will be turning
        # if the highest speed is greater than one, we then make the largest value equal one, while keeping the ratios the same
        max = frspeed
        if flspeed > max:
            max = flspeed  # would use elif, but we can't gurantee that only one value will be larger than the front right wheel speed
        if rlspeed > max:
            max = rlspeed
        if rrspeed > max:
            max = rrspeed

        if max > 1:
            frspeed /= max
            flspeed /= max
            rlspeed /= max
            rrspeed /= max

        # make wheels turn and spin at the speeds and angles calculated above
        self.turnWheel(self.leftFrontSwerveModule, flangle, flspeed)
        self.turnWheel(self.leftRearSwerveModule, rlangle, rlspeed)
        self.turnWheel(self.rightFrontSwerveModule, frangle, frspeed)
        self.turnWheel(self.rightRearSwerveModule, rrangle, rrspeed)

    def reset(self):
        self.navX.reset()
        #self.gyro.calibrate()

        self.leftFrontDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.leftFrontSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)

        self.leftRearDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.leftRearSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)

        self.rightFrontDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightFrontSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)

        self.rightRearDirection.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)
        self.rightRearSpeed.setSelectedSensorPosition(0.0, constants.kPIDLoopIdx, constants.ktimeoutMs)