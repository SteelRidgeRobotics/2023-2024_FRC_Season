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
            wpilib.SmartDashboard.putNumber("RevComp", conversions.giveRevCompensation(currentAngle, direction))
            wpilib.SmartDashboard.putNumber("REVOLUTIONS", conversions.getRevolutions(currentAngle))
            wpilib.SmartDashboard.putNumber("Given Angle", conversions.getclosest(currentAngle, direction, magnitude)[0])
            wpilib.SmartDashboard.putNumber("Current Angle", currentAngle)

    def translate(self, direction: float, magnitude: float):
        self.turnWheel(self.leftFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.leftRearSwerveModule, direction, magnitude)
        self.turnWheel(self.rightFrontSwerveModule, direction, magnitude)
        self.turnWheel(self.rightRearSwerveModule, direction, magnitude)

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

    def showWheelStats(self):

        wpilib.SmartDashboard.putNumber(" LF Steering ", self.leftFrontSwerveModule.getCurrentAngle())
        wpilib.SmartDashboard.putNumber(" LF CAN ", self.leftFrontSwerveModule.getAbsPos())
        wpilib.SmartDashboard.putNumber(" LF Offset ", self.leftFrontSwerveModule.steeringOffset)
        wpilib.SmartDashboard.putNumber(" LF Speed ", self.leftFrontSwerveModule.getVelocity())

        wpilib.SmartDashboard.putNumber(" LR Steering ", self.leftRearSwerveModule.getCurrentAngle())
        wpilib.SmartDashboard.putNumber(" LR CAN ", self.leftRearSwerveModule.getAbsPos())
        wpilib.SmartDashboard.putNumber(" LR Offset ", self.leftRearSwerveModule.steeringOffset)
        wpilib.SmartDashboard.putNumber(" LR Speed ", self.leftRearSwerveModule.getVelocity())

        wpilib.SmartDashboard.putNumber(" RF Steering ", self.rightFrontSwerveModule.getCurrentAngle())
        wpilib.SmartDashboard.putNumber(" RF CAN ", self.rightFrontSwerveModule.getAbsPos())
        wpilib.SmartDashboard.putNumber(" RF Offset ", self.rightFrontSwerveModule.steeringOffset)
        wpilib.SmartDashboard.putNumber(" RF Speed ", self.rightFrontSwerveModule.getVelocity())

        wpilib.SmartDashboard.putNumber(" RR Steering ", self.rightRearSwerveModule.getCurrentAngle())
        wpilib.SmartDashboard.putNumber(" RR CAN ", self.rightRearSwerveModule.getAbsPos())
        wpilib.SmartDashboard.putNumber(" RR Offset ", self.rightRearSwerveModule.steeringOffset)
        wpilib.SmartDashboard.putNumber(" RR Speed ", self.rightRearSwerveModule.getVelocity())

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

        frangle = math.atan2(b, c) * 180 / math.pi
        flangle = math.atan2(b, d) * 180 / math.pi
        rlangle = math.atan2(a, d) * 180 / math.pi
        rrangle = math.atan2(a, c) * 180 / math.pi

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