from commands2 import CommandScheduler, TimedCommandRobot
from robotcontainer import RobotContainer
from wpilib import run
from wpimath.geometry import Rotation2d

class MyRobot(TimedCommandRobot):
    def robotInit(self):

        self.container = RobotContainer()
        self.autoCommand = self.container.getAutonomousCommand()

    def robotPeriodic(self):

        CommandScheduler.getInstance().run()

        self.container.getSwerveDrive().updateWheelPositions()
        self.container.getSwerveDrive().updateOdometry()

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
    run(MyRobot)
