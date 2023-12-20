import wpilib
from commands2 import CommandScheduler, TimedCommandRobot
from robotcontainer import RobotContainer


class Larry(TimedCommandRobot):
    field = wpilib.Field2d()

    def robotInit(self):

        self.container = RobotContainer()
        self.autoCommand = self.container.getAutonomousCommand()
        self.container.getSwerveDrive().resetOdometry()
        self.container.getSwerveDrive().updateFieldPose(self.field)

    def robotPeriodic(self):

        CommandScheduler.getInstance().run()
        self.container.getSwerveDrive().updateOdometry()
        self.container.getSwerveDrive().sendVoltagesToSmartDashboard()
        self.container.getSwerveDrive().updateFieldPose(self.field)

    def autonomousInit(self):
        self.autoCommand = self.container.getAutonomousCommand()

        if self.autoCommand:
            self.autoCommand.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):

        if self.autoCommand:
            self.autoCommand.cancel()

        self.drivingMode = self.container.getDrivingMode()
        self.container.getSwerveDrive().setDefaultCommand(self.drivingMode)
        
    def teleopPeriodic(self):
        if self.drivingMode != self.container.getDrivingMode():
            self.container.getSwerveDrive().removeDefaultCommand()
            self.container.getSwerveDrive().setDefaultCommand(self.container.getDrivingMode())
            
            self.drivingMode = self.container.getDrivingMode()

    def testInit(self):

        CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(Larry)
