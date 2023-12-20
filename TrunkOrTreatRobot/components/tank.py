from constants import *
from ctre import FeedbackDevice, NeutralMode, TalonFX, TalonFXControlMode
from wpilib import SmartDashboard

class Tank4MDrive:
    leftFollower = TalonFX
    rightFollower = TalonFX
    leftLeader = TalonFX
    rightLeader = TalonFX

    def setup(self) -> None:
        # TODO: Have James fix this
        self.leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kTimeoutMs)
        self.rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, kTimeoutMs)

        self.leftFollower.setNeutralMode(NeutralMode.Brake)
        self.leftLeader.setNeutralMode(NeutralMode.Brake)
        self.rightFollower.setNeutralMode(NeutralMode.Brake)
        self.rightLeader.setNeutralMode(NeutralMode.Brake)

        self.leftLeader.configNominalOutputForward(0, kTimeoutMs)
        self.rightLeader.configNominalOutputForward(0, kTimeoutMs)
        self.leftLeader.configNominalOutputReverse(0, kTimeoutMs)
        self.rightLeader.configNominalOutputReverse(0, kTimeoutMs)

        self.leftLeader.configPeakOutputForward(kMaxSpeed, kTimeoutMs)
        self.rightLeader.configPeakOutputForward(kMaxSpeed, kTimeoutMs)
        self.leftLeader.configPeakOutputReverse(kMaxSpeed, kTimeoutMs)
        self.rightLeader.configPeakOutputReverse(kMaxSpeed, kTimeoutMs)

        self.leftFollower.selectProfileSlot(0, 0)
        self.rightFollower.selectProfileSlot(0, 0)
        self.leftLeader.selectProfileSlot(0, 0)
        self.rightLeader.selectProfileSlot(0, 0)

        self.leftLeader.configMotionCruiseVelocity(kmotorCruiseVelocity, kTimeoutMs)
        self.rightLeader.configMotionCruiseVelocity(kmotorCruiseVelocity, kTimeoutMs)
        self.leftLeader.configMotionAcceleration(kmotorAcceleration, kTimeoutMs)
        self.rightLeader.configMotionAcceleration(kmotorAcceleration, kTimeoutMs)

        self.rightFollower.setInverted(True)
        self.rightLeader.setInverted(True)

        self.leftFollower.follow(self.leftLeader)
        self.rightFollower.follow(self.rightLeader)

    def setMags(self,leftMag: float, rightMag: float) -> None:
        self.leftMag = leftMag
        self.rightMag = rightMag

    def execute(self) -> None:
        self.leftLeader.set(TalonFXControlMode.PercentOutput, self.leftMag)
        self.rightLeader.set(TalonFXControlMode.PercentOutput, self.rightMag)
        SmartDashboard.putNumber("leftMag", self.leftMag)
        SmartDashboard.putNumber("rightMag", self.rightMag)

        