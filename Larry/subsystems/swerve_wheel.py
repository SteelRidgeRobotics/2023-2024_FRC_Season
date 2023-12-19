from math import fabs, pi

import ctre
from ctre import TalonFXConfiguration
from constants import *
from ctre.sensors import CANCoder, SensorInitializationStrategy
from wpilib import RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveWheel():
    def __init__(self, directionMotor: ctre.TalonFX, driveMotor: ctre.TalonFX, 
                 CANCoder: CANCoder, MagOffset: float, name: str) -> None:

        self.directionMotor = directionMotor
        self.driveMotor = driveMotor

        self.directionMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)
        self.driveMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)

        self.directionMotor.config_kF(0, kF, ktimeoutMs)
        self.driveMotor.config_kF(0, kF, ktimeoutMs)

        self.directionMotor.config_kP(0, kP, ktimeoutMs)
        self.driveMotor.config_kP(0, kP, ktimeoutMs)

        self.directionMotor.config_kI(0, kI, ktimeoutMs)
        self.driveMotor.config_kI(0, kI, ktimeoutMs)

        self.directionMotor.config_kD(0, kD, ktimeoutMs)
        self.driveMotor.config_kD(0, kD, ktimeoutMs)

        self.directionMotor.config_IntegralZone(0, kIzone, ktimeoutMs)
        self.driveMotor.config_IntegralZone(0, kIzone, ktimeoutMs)

        # MOTOR CONFIG
        self.directionMotor.configNominalOutputForward(0, ktimeoutMs)
        self.driveMotor.configNominalOutputForward(0, ktimeoutMs)

        self.directionMotor.configNominalOutputReverse(0, ktimeoutMs)
        self.driveMotor.configNominalOutputReverse(0, ktimeoutMs)

        self.directionMotor.configPeakOutputForward(1, ktimeoutMs)
        self.driveMotor.configPeakOutputForward(1, ktimeoutMs)

        self.directionMotor.configPeakOutputReverse(-1, ktimeoutMs)
        self.driveMotor.configPeakOutputReverse(-1, ktimeoutMs)

        self.directionMotor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)
        self.driveMotor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)

        self.directionMotor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)
        self.driveMotor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)

        """
        directionConfig = TalonFXConfiguration()
        directionConfig.nominalOutputForward = 0
        directionConfig.nominalOutputReverse = 0
        directionConfig.peakOutputForward = 1
        directionConfig.peakOutputReverse = -1
        directionConfig.motionCruiseVelocity = kcruiseVel
        directionConfig.motionAcceleration = kcruiseAccel

        driveConfig = TalonFXConfiguration()
        driveConfig.nominalOutputForward = 0
        driveConfig.nominalOutputReverse = 0
        driveConfig.peakOutputForward = 1
        driveConfig.peakOutputReverse = -1
        driveConfig.motionCruiseVelocity = kcruiseVel
        driveConfig.motionAcceleration = kcruiseAccel

        self.directionMotor.configAllSettings(directionConfig)
        self.driveMotor.configAllSettings(driveConfig)
        """

        self.directionMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)
        self.driveMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)

        self.directionMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Brake)

        self.directionMotor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.driveMotor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

        self.CANCoder = CANCoder
        self.CANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, ktimeoutMs)
        self.CANCoder.configSensorDirection(True, ktimeoutMs)
        self.CANCoder.configMagnetOffset(MagOffset)

        self.directionTargetPos = 0.0
        self.directionTargetAngle = 0.0
        self.isInverted = False

        self.position = SwerveModulePosition()

        self.name = name

        self.feedForward = SimpleMotorFeedforwardMeters(kS=0, kV=1, kA=1) # kS = Vintercept btw from the whitepaper (see Tuning Larry Google Doc in Programming)
        self.lastAngle = 0

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        desiredState = self.optimizeAngle(desiredState, Rotation2d.fromDegrees(self.getCurrentAngle()))

        velocity = mpsToFalcon(desiredState.speed, klarryWheelSize, ksteeringGearRatio)

        self.driveMotor.set(ctre.ControlMode.Velocity, velocity, ctre.DemandType.ArbitraryFeedForward,
                            self.feedForward.calculate(desiredState.speed))
        
        self.driveMotor.getSimCollection().addIntegratedSensorPosition(int(velocity))

        if fabs(desiredState.speed) <= klarryMaxSpeed * 0.01:
            angle = self.lastAngle
        else:
            angle = desiredState.angle.degrees() # Prevents jittering

        self.directionMotor.set(ctre.ControlMode.Position,
            degreesToFalcon(angle, ksteeringGearRatio))
        self.directionMotor.getSimCollection().setIntegratedSensorRawPosition(int(degreesToFalcon(angle, ksteeringGearRatio)))
        self.lastAngle = angle

    def optimizeAngle(self, desiredState: SwerveModuleState, currentAngle: Rotation2d) -> SwerveModuleState:
        targetAngle = self.placeAngleInScope(currentAngle.degrees(), desiredState.angle.degrees())
        targetSpeed = desiredState.speed
        delta = targetAngle - currentAngle.degrees()
        if (fabs(delta) > 90):
            targetSpeed *= -1
            if delta > 90:
                targetAngle -= 180
            else:
                targetAngle += 180
        return SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle))

    def placeAngleInScope(self, scopeReference: float, newAngle: float) -> float:
        lowerBound = None
        upperBound = None

        lowerOffset = scopeReference % 360
        if lowerOffset >= 0:
            lowerBound = scopeReference - lowerOffset
            upperBound = scopeReference + (360 - lowerOffset)
        else:
            upperBound = scopeReference - lowerOffset
            lowerBound = scopeReference - (360 + lowerOffset)
        while newAngle < lowerBound:
            newAngle += 360
        while newAngle > upperBound:
            newAngle -= 360
        if newAngle - scopeReference > 180:
            newAngle -= 360
        elif newAngle - scopeReference < -180:
            newAngle += 360
        return newAngle

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

        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, input * slowdownMult)
        self.driveMotor.getSimCollection().addIntegratedSensorPosition(int(input * kmaxWheelSpeed))

    def stop(self):
        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)

    def getPosition(self) -> SwerveModulePosition:
        return self.position
    
    def updatePostion(self) -> None:
        currentAngle = self.getCurrentAngle()
        currentPos = self.driveMotor.getSelectedSensorPosition() / ksteeringGearRatio
        self.position = SwerveModulePosition(posToMeters(currentPos), Rotation2d.fromDegrees(currentAngle))

    def getCurrentAngle(self):
        return (self.directionMotor.getSelectedSensorPosition() / ksteeringGearRatio) * (360 / 2048)
    
    def isAtCorrectAngle(self, error :float = 1.0) -> bool:
        angle = self.getCurrentAngle()
        return self.directionTargetAngle - abs(error) <= angle <= self.directionTargetAngle + abs(error)
    
def posToMeters(pos) -> float:
    return pos * (((klarryWheelSize / 2) * pi * 2) / (ksteeringGearRatio * 2048.0))

def mpsToFalcon(velocity: float, circumference: float, gearRatio: float) -> float:
    wheelRPM = ((velocity * 60) / circumference)
    wheelVelocity = rpmToFalcon(wheelRPM, gearRatio)
    return wheelVelocity

def rpmToFalcon(rpm: float, gearRatio: float) -> float:
    motorRPM = rpm * gearRatio
    sensorCounts = motorRPM * (2048.0 / 600.0)
    return sensorCounts

def degreesToFalcon(degrees: float, gearRatio: float) -> float:
    return degrees / (360.0 / (gearRatio * 2048.0))
    