from wpilib import TimedRobot
from commands2 import CommandScheduler
from commands2 import Command
from RobotContainer import *
from CTREConfigs import *


class Robot(TimedRobot):
    ctre_configs = CTREConfigs()

    autonomous_command = Command

    def robotInit(self):
        self.robot_container = RobotContainer()

    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.autonomous_command = self.robot_container.get_autonomous_command()

        # Schedule the autonomous command (example)
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()

    def teleopPeriodic(self):
        pass

    def testInit(self):
        # Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass
