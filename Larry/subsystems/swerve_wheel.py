from math import fabs, pi

import ctre
from constants import *
from ctre.sensors import CANCoder
from wpilib import RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition


class SwerveWheel():
    def __init__(self, directionMotor: ctre.TalonFX, speedMotor: ctre.TalonFX, 
                 CANCoder: CANCoder, MagOffset: float, name: str) -> None:

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

        self.directionTargetPos = 0.0
        self.directionTargetAngle = 0.0
        self.isInverted = False

        self.position = SwerveModulePosition()

        self.name = name

    def turnToOptimizedAngle(self, desiredAngle) -> None:
        """
        Takes the desired angle for the motor, then:
        1. Turn to the opposite angle and invert the speed motor, and
        2. Turns the wheel clockwise or counter clockwise

        :params desiredAngle: The angle that we want the wheel to turn to.
        """
        desiredAngle %= 360 # just making sure ;) (0-359)

        angleDist = fabs(desiredAngle - self.directionTargetAngle)

        # If the angleDist is more than 90 and less than 270, add 180 to the angle and %= 360 to get oppositeAngle.
        if (angleDist > 90 and angleDist < 270):
            targetAngle = (desiredAngle + 180) % 360
            self.isInverted = True
        
        # Else, then like, idk, just go to it??? smh
        else:
            targetAngle = desiredAngle
            self.isInverted = False

        # Before, to move the motor to the right spot, we would take the angle, convert that into talonFX units, then add (the amount of revolutions * 2048), then multiple everything by the motors gear ratio
        # However, to avoid having to deal with revolution compensation (which caused some issues), we now get the degree change, convert to motor units, then add or subtract depending on the direction we're rotating
        targetAngleDist = fabs(targetAngle - self.directionTargetAngle)

        # When going from x angle to 0, the robot will try and go "the long way around" to the angle. This just checks to make sure we're actually getting the right distance
        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        changeInTalonUnits = targetAngleDist / (360/2048)

        # Now that we have the correct angle, we figure out if we should rotate counterclockwise or clockwise
        angleDiff = targetAngle - self.directionTargetAngle

        # Accounting if the angleDiff is negative
        if angleDiff < 0:
            angleDiff += 360

        # If angleDiff is greater than 180, go counter-clockwise (ccw is positive for talonFX, and vice versa)
        if angleDiff > 180:
            self.directionTargetPos -= changeInTalonUnits

        # Else, go clockwise
        else:
            self.directionTargetPos += changeInTalonUnits

        self.directionTargetAngle = targetAngle

        if kDebug:
            SmartDashboard.putNumber(self.name + " dirTargetAngle", self.directionTargetAngle)
            SmartDashboard.putBoolean(self.name + " Inverted?", self.isInverted)

        # Now we can actually turn the motor after like 60 lines lmao
        self.directionMotor.set(ctre.TalonFXControlMode.MotionMagic, self.directionTargetPos * ksteeringGearRatio)
        self.directionMotor.getSimCollection().setIntegratedSensorRawPosition(int(self.directionTargetPos * ksteeringGearRatio))

    def CANtoTalon(self):
        self.directionMotor.setSelectedSensorPosition(ksteeringGearRatio * (self.CANCoder.getAbsolutePosition() * (2048 / 360)), 0, ktimeoutMs)
    
    def move(self, input: float, slowdownWhenFar: bool=True) -> None:
        if self.isInverted:
            input *= -1

        # Slows down motor at a curve depending on how far the motor angle is from the target angle.
        # Don't worry about the actual equation, I just eyeballed the numbers lmao
        angleDiff = self.directionTargetAngle - self.getCurrentAngle()
        angleDiff = (angleDiff + 180) % 360 - 180

        slowdownMult = max(0, min(1.0, (-(3.14514 / 112006) * (angleDiff ** 2)) + 1))
        if not RobotBase.isReal() or not slowdownWhenFar:
            slowdownMult = 1

        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, input * slowdownMult)
        self.speedMotor.getSimCollection().addIntegratedSensorPosition(int(input * kmaxWheelSpeed))

    def stop(self):
        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)

    def getPosition(self) -> SwerveModulePosition:
        return self.position
    
    def updatePostion(self) -> None:
        currentAngle = self.getCurrentAngle()
        currentPos = self.speedMotor.getSelectedSensorPosition()
        self.position = SwerveModulePosition(posToMeters(currentPos), Rotation2d.fromDegrees(currentAngle))

    def getCurrentAngle(self):
        return (self.directionMotor.getSelectedSensorPosition() / ksteeringGearRatio) * (360 / 2048)
    
    def isAtCorrectAngle(self, error :float = 1.0) -> bool:
        angle = self.getCurrentAngle()
        return self.directionTargetAngle - abs(error) <= angle <= self.directionTargetAngle + abs(error)
    
def posToMeters(pos) -> float:
    return pos * (((klarryWheelSize / 2) * pi * 2) / (ksteeringGearRatio * 2048.0))
    