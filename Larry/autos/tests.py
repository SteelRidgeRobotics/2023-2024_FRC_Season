from autos.base import AutoBase
from commands2 import CommandBase, InstantCommand
from constants import *
from pathplannerlib import PathPlanner
from subsystems.swerve_drive import SwerveDrive
from wpilib import Timer
from wpimath.geometry import Translation2d, Rotation2d

class TestForward(AutoBase):
    """
    Moves the robot 1 meter forward using a PathPlanner trajectory.
    """

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__(swerve)

        path = PathPlanner.loadPath("TestForward", kdefaultPathConstraints)
        command1 = self.baseSwerveCommand(path)
        initialState = command1.pose()

        self.addCommands(
            InstantCommand(lambda: swerve.navX.reset()),
            InstantCommand(lambda: swerve.resetOdometry()),
            command1
        )

class TestForwardNoPP(CommandBase):
    """
    Moves the robot 1 meter forward
    """
    timer = Timer()

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__()

        self.swerve = swerve

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.swerve.drive(Translation2d(10, 0), Rotation2d(), fieldRelative=False)

    def end(self, interrupted: bool) -> None:
        self.swerve.stopAllMotors()
        self.timer.stop()

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(1)
