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

        # self.directionMotor.setSelectedSensorPosition(0.0, 0, ktimeoutMs)

        self.directionMotor.config_kF(0, kF, ktimeoutMs)

        self.directionMotor.config_kP(0, kP, ktimeoutMs)

        self.directionMotor.config_kI(0, kI, ktimeoutMs)

        self.directionMotor.config_kD(0, kD, ktimeoutMs)

        self.directionMotor.config_IntegralZone(0, kIzone, ktimeoutMs)

        # MOTOR CONFIG
        self.directionMotor.configNominalOutputForward(0, ktimeoutMs)
        self.directionMotor.configNominalOutputReverse(0, ktimeoutMs)

        self.directionMotor.configPeakOutputForward(1, ktimeoutMs)
        self.directionMotor.configPeakOutputReverse(-1, ktimeoutMs)

        self.directionMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx)

        self.directionMotor.configMotionCruiseVelocity(kcruiseVel, ktimeoutMs)
        self.directionMotor.configMotionAcceleration(kcruiseAccel, ktimeoutMs)

        self.directionMotor.setNeutralMode(ctre.NeutralMode.Brake)

        self.directionMotor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

        # CAN Coder
        self.CANCoder = CANCoder
        
        self.CANCoder.configSensorInitializationStrategy(ctre.SensorInitializationStrategy.BootToAbsolutePosition, ktimeoutMs)
        self.CANCoder.configSensorDirection(True, ktimeoutMs)
        self.CANCoder.configMagnetOffset(MagOffset)

        wpilib.SmartDashboard.putNumber(" P -", kP)
        wpilib.SmartDashboard.putNumber(" I -", kI)
        wpilib.SmartDashboard.putNumber(" D -", kD)
        wpilib.SmartDashboard.putNumber(" F -", kF)
        # wpilib.SmartDashboard.putNumber(" Sensor Position -", self.directionMotor.getSelectedSensorPosition())
        self.notTurning = True

        self.steeringOffset = 0.0

        self.moffset = manualOffset

    # this is our testing turn method
    def turn(self, set_point: float):

        self.notTurning = False
        current_pos = self.directionMotor.getSelectedSensorPosition()
        # convert manual offset angle into TalonFX Units
        offset = convertDegreesToTalonFXUnits(self.moffset) * ksteeringGearRatio
        self.directionMotor.set(ctre.TalonFXControlMode.MotionMagic, 
                                int(set_point + offset))

    def getAbsPos(self) -> float:

        return self.CANCoder.getAbsolutePosition()

    def CANtoTalon(self):

        self.steeringOffset = (ksteeringGearRatio * 
                               convertDegreesToTalonFXUnits(self.getAbsPos()))
        
        # self.directionMotor.configIntegratedSensorOffset(self.steeringOffset, ktimeoutMs)
        self.directionMotor.setSelectedSensorPosition(self.steeringOffset, 0, ktimeoutMs)
        
    def isNotinMotion(self) -> bool:

        if self.directionMotor.getActiveTrajectoryVelocity() == 0.0:
            self.notTurning = True
        else:
            self.notTurning = False
        return self.notTurning

    def move(self, joystick_input: float):
        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.2 * joystick_input)

    def stopAllMotors(self):
        self.directionMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)
        self.speedMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.0)
        self.directionMotor.setNeutralMode(ctre.NeutralMode.Coast)

    def getCurrentAngle(self):
        return convertTalonFXUnitsToDegrees(self.directionMotor.getSelectedSensorPosition())

    def getVelocity(self):
        return self.speedMotor.getSelectedSensorVelocity()

    def resetToOrigin(self):
        self.turn(0 + self.offset)