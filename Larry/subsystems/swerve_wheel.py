# import commands2
import ctre
import wpilib
from conversions import *
from constants import *

class SwerveWheel():
    def __init__(self, directionMotor: ctre.TalonFX, speedMotor: ctre.TalonFX, 
                 CANCoder: ctre.CANCoder, MagOffset: float, 
                 manualOffset: float) -> None:
        # super().__init__()
        self.directionMotor = directionMotor
        self.speedMotor = speedMotor

        self.directionMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)
        self.speedMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)

        self.directionMotor.config_kF(0, kF, ktimeoutMs)
        self.speedMotor.config_kF(0, kF, ktimeoutMs)

        self.directionMotor.config_kP(0, kP, ktimeoutMs)
        self.speedMotor.config_kP(0, kP, ktimeoutMs)

        self.directionMotor.config_kI(0, kI, ktimeoutMs)
        self.speedMotor.config_kI(0, kI, ktimeoutMs)

        self.directionMotor.config_kD(0, kD, ktimeoutMs)
        self.speedMotor.config_kD(0, kD, ktimeoutMs)

        self.directionMotor.config_IntegralZone(0, kIzone, ktimeoutMs)
        self.speedMotor.config_IntegralZone(0, kIzone, ktimeoutMs)

        # MOTOR CONFIG
        self.directionMotor.configNominalOutputForward(0, ktimeoutMs)
        self.speedMotor.configNominalOutputForward(0, ktimeoutMs)

        self.directionMotor.configNominalOutputReverse(0, ktimeoutMs)
        self.speedMotor.configNominalOutputReverse(0, ktimeoutMs)

        self.directionMotor.configPeakOutputForward(1, ktimeoutMs)
        self.speedMotor.configPeakOutputForward(1, ktimeoutMs)

        self.directionMotor.configPeakOutputReverse(-1, ktimeoutMs)
        self.speedMotor.configPeakOutputReverse(-1, ktimeoutMs)

        self.directionMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)
        self.speedMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)

        self.directionMotor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)
        self.speedMotor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)

        self.directionMotor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)
        self.speedMotor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)

        self.directionMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.speedMotor.setNeutralMode(ctre.NeutralMode.Brake)

        self.directionMotor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.speedMotor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

        self.CANCoder = CANCoder
        
        self.CANCoder.configSensorInitializationStrategy(ctre.SensorInitializationStrategy.BootToAbsolutePosition, ktimeoutMs)
        self.CANCoder.configSensorDirection(True, ktimeoutMs)
        self.CANCoder.configMagnetOffset(MagOffset)

        wpilib.SmartDashboard.putNumber(" P -", kP)
        wpilib.SmartDashboard.putNumber(" I -", kI)
        wpilib.SmartDashboard.putNumber(" D -", kD)
        wpilib.SmartDashboard.putNumber(" F -", kF)

    def turnToOptimizedAngle(self, desiredAngle) -> bool:
        """
        Takes the desired angle for the motor and calculates if we should:
        A: Turn clockwise,
        B: Turn counterclockwise,
        C: Turn to the opposite angle and invert our magnitude

        The function then executes to that angle.

        Do note that if C, the magnitude will have to be inverted seperately outside of this method. Sorry, hopefully you read this.

        :params desiredAngle: The angle that we want the wheel to turn to.

        :returns: True if magnitude needs to be inverted.
        """
        desiredAngle %= 360 # just making sure ;) (0-359)

        currentAngle = self.getCurrentAngle() % 360 #0 - 359

        # Distance from desiredAngle to currentAngle # 0 - 359
        angleDist = abs(desiredAngle - currentAngle)

        # If the angleDist is more than 90 and less than 270, add 180 to the angle and %= 360 to get oppositeAngle. NOTE: MAGNITUDE WILL NEED TO BE INVERTED TO COMPENSATE
        if (angleDist > 90 and angleDist < 270):
            targetAngle = (desiredAngle + 180) % 360
            invertMagnitude = True
        
        # Else, then like, idk, just go to it??? smh
        else:
            targetAngle = desiredAngle
            invertMagnitude = False

        
        # Now that we have the correct angle, we figure out if we should rotate counterclockwise or clockwise
        angleDistButThisTimeItsNotAbsLol = currentAngle - targetAngle

        # Before, to move the motor to the right spot, we take the angle, convert that into talonFX units, then add (the amount of revolutions * 2048), then multiple everything by the motors gear ratio
        # However, to avoid having to deal with revolution compensation (which caused some issues), we now get the degree change, convert to motor units, then add or subtract depending on the direction we're rotating
        motorPos = self.directionMotor.getSelectedSensorPosition() / ksteeringGearRatio
        changeInTalonUnits = abs(currentAngle - targetAngle) * (360/2048)

        # If that long ass variable is greater than -180 and less than 0, go clockwise (cw = negative motor units)
        if angleDistButThisTimeItsNotAbsLol >= -180 and angleDistButThisTimeItsNotAbsLol < 0:
            newMotorPos = motorPos - changeInTalonUnits

        # Else, go counter clockwise (ccw = positive motor units)
        else:
            newMotorPos = motorPos + changeInTalonUnits

        # Now we can actually turn the motor after like 50 lines lmao
        self.directionMotor.set(ctre.TalonFXControlMode.MotionMagic, newMotorPos * ksteeringGearRatio)

        # If we have to invert the magnitude, return True, else, return False
        return invertMagnitude

    def getRevolutions(self) -> int:
        pos = self.directionMotor.getSelectedSensorPosition() / ksteeringGearRatio
        return (pos - (pos % 2048)) / 2048

    def CANtoTalon(self):
        self.directionMotor.setSelectedSensorPosition(ksteeringGearRatio * (self.CANCoder.getAbsolutePosition() * (2048 / 360)), 0, ktimeoutMs)
    
    def move(self, joystick_input: float):
        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, (1.0) if kMaxSwerveSpeed else (0.33) * joystick_input)

    def stopAllMotors(self):
        self.directionMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)
        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)
        self.directionMotor.setNeutralMode(ctre.NeutralMode.Coast)

    def getCurrentAngle(self):
        return (self.directionMotor.getSelectedSensorPosition() / ksteeringGearRatio) * (360 / 2048)

    def getVelocity(self):
        return self.speedMotor.getSelectedSensorVelocity()