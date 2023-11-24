import commands2
import wpilib

from robotcontainer import RobotContainer


# vision setup

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):

        self.container = RobotContainer()
        self.autoCommand = self.container.getAutonomousCommand()

    def robotPeriodic(self):

        commands2.CommandScheduler.getInstance().run()

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

        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)