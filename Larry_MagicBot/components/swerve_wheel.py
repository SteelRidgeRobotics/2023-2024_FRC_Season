from constants import *
from ctre import FeedbackDevice, NeutralMode, TalonFX, TalonFXControlMode
from ctre.sensors import CANCoder, SensorInitializationStrategy
from math import fabs
import wpilib

class SwerveWheel:
    speed_motor: TalonFX
    direction_motor: TalonFX
    cancoder: CANCoder

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """
        self.direction_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)
        self.speed_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)

        self.direction_motor.config_kF(0, kF, ktimeoutMs)
        self.speed_motor.config_kF(0, kF, ktimeoutMs)

        self.direction_motor.config_kP(0, kP, ktimeoutMs)
        self.speed_motor.config_kP(0, kP, ktimeoutMs)

        self.direction_motor.config_kI(0, kI, ktimeoutMs)
        self.speed_motor.config_kI(0, kI, ktimeoutMs)

        self.direction_motor.config_kD(0, kD, ktimeoutMs)
        self.speed_motor.config_kD(0, kD, ktimeoutMs)

        self.direction_motor.config_IntegralZone(0, kIzone, ktimeoutMs)
        self.speed_motor.config_IntegralZone(0, kIzone, ktimeoutMs)

        self.direction_motor.configNominalOutputForward(0, ktimeoutMs)
        self.speed_motor.configNominalOutputForward(0, ktimeoutMs)

        self.direction_motor.configNominalOutputReverse(0, ktimeoutMs)
        self.speed_motor.configNominalOutputReverse(0, ktimeoutMs)

        self.direction_motor.configPeakOutputForward(1, ktimeoutMs)
        self.speed_motor.configPeakOutputForward(1, ktimeoutMs)

        self.direction_motor.configPeakOutputReverse(-1, ktimeoutMs)
        self.speed_motor.configPeakOutputReverse(-1, ktimeoutMs)

        self.direction_motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)
        self.speed_motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)

        self.direction_motor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)
        self.speed_motor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)

        self.direction_motor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)
        self.speed_motor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)

        self.direction_motor.setNeutralMode(NeutralMode.Brake)
        self.speed_motor.setNeutralMode(NeutralMode.Brake)

        self.direction_motor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.speed_motor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

        self.speed_motor.setInverted(True)
        
        self.cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, ktimeoutMs)
        self.cancoder.configSensorDirection(True, ktimeoutMs)

        self.directionTargetPos = 0.0
        self.directionTargetAngle = 0.0
        self.isInverted = False
        self.desiredAngle = 0
        self.desiredSpeed = 0
        self.stopped = False

    """
    CONTROL METHODS
    """
    def setDesiredAngle(self, angle: int) -> None:
        """
        Sets the desired angle we want the direction motor to turn to
        when the execute command is ran.
        """
        self.desiredAngle = angle

    def setDesiredSpeed(self, speed: int) -> None:
        self.desiredSpeed = max(-1, min(1, speed))

    def stopWheel(self) -> None:
        self.speed_motor.set(TalonFXControlMode.PercentOutput, 0)
        self.direction_motor.set(TalonFXControlMode.PercentOutput, 0)
        self.direction_motor.setNeutralMode(NeutralMode.Coast)

        # Prevents SmartDashboard desync
        if kDebug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.getDeviceID()) + " Mag", 0)

        self.stopped = True

    """
    EXECUTE
    """
    def execute(self) -> None:
        if self.stopped: # Stops angle from updating when stopped.
            self.stopped = False
            return

        """
        CHANGING DIRECTION
        """
        self.desiredAngle %= 360 # just making sure ;) (0-359)

        angleDist = fabs(self.desiredAngle - self.directionTargetAngle)

        # If the angleDist is more than 90 and less than 270, add 180 to the angle and %= 360 to get oppositeAngle.
        if (angleDist > 90 and angleDist < 270):
            targetAngle = (self.desiredAngle + 180) % 360
            self.isInverted = True
        
        # Else, then like, idk, just go to it??? smh
        else:
            targetAngle = self.desiredAngle
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
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.getDeviceID()) + " dirTargetAngle", self.directionTargetAngle)
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.getDeviceID()) + " dirTargetPos", self.directionTargetPos)
            wpilib.SmartDashboard.putBoolean(str(self.speed_motor.getDeviceID()) + " Inverted?", self.isInverted)

        # Now we can actually turn the motor after like 60 lines lmao
        self.direction_motor.set(TalonFXControlMode.MotionMagic, self.directionTargetPos * ksteeringGearRatio)

        """
        ACTUALLY MOVING
        """
        if self.isInverted:
            self.desiredSpeed *= -1

        # Slows down motor at a curve depending on how far the motor angle is from the target angle.
        # Don't worry about the actual equation, I just eyeballed the numbers lmao
        angleDiff = self.directionTargetAngle - (self.direction_motor.getSelectedSensorPosition() / ksteeringGearRatio) * (360 / 2048)
        angleDiff = (angleDiff + 180) % 360 - 180

        slowdownMult = max(-1.0, min(1.0, (-(3.14514 / 112006) * (angleDiff ** 2)) + 1))
        if not wpilib.RobotBase.isReal():
            slowdownMult = 1

        self.speed_motor.set(TalonFXControlMode.PercentOutput, max(-1, min(1, self.desiredSpeed * slowdownMult)))

        if kDebug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.getDeviceID()) + " Mag", max(-1, min(1, self.desiredSpeed * slowdownMult)))
        