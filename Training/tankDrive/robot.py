import wpilib
import ctre

class TrainingBot(wpilib.TimedRobot):

    def robotInit(self):

        self.FLMotor = ctre.TalonFX(0)
        self.BLMotor = ctre.TalonFX(1)
        self.FRMotor = ctre.TalonFX(2)
        self.BRMotor = ctre.TalonFX(3)

        self.BLMotor.follow(self.FLMotor)
        self.BRMotor.follow(self.FRMotor)

        self.FRMotor.setInverted(True)

        self.dawg = wpilib.XboxController(0)

    def teleopPeriodic(self):

        self.left = -(self.dawg.getLeftY())
        self.right = -(self.dawg.getRightY())

        self.FLMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.2 * self.left)
        self.FRMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.2 * self.right)

    def autonomousInit(self):

        self.bigBen = wpilib.Timer()
        self.bigBen.reset()
        self.bigBen.start()

        wpilib.SmartDashboard.putString("Phase", "Initialized")

    def autonomousPeriodic(self):

        self.timerValue = self.bigBen.get()

        if self.timerValue <= 2:

            self.FLMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.25)
            self.FRMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.25)
            wpilib.SmartDashboard.putString("Phase", "Forward")

        elif self.timerValue <= 3.5:

            self.FLMotor.set(ctre.TalonFXControlMode.PercentOutput, -0.125)
            self.FRMotor.set(ctre.TalonFXControlMode.PercentOutput, 0.125)
            wpilib.SmartDashboard.putString("Phase", "Turning")

        else:

            self.FLMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
            self.FRMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
            wpilib.SmartDashboard.putString("Phase", "Stopped")

        wpilib.SmartDashboard.putNumberArray("Motor Percentages", [self.FLMotor.getMotorOutputPercent(), self.FRMotor.getMotorOutputPercent()])
        wpilib.SmartDashboard.putNumber("Big Ben Value", self.timerValue)


if __name__ == "__main__":

    wpilib.run(TrainingBot)