import commands2
import wpilib
from subsystems.swerve_drive import SwerveDrive

class ChargeStation(commands2.CommandBase):

    def __init__(self, swerveDrive: SwerveDrive):

        super().__init__()

        self.drive = swerveDrive
        self.timer = wpilib.Timer()

        self.addRequirements([self.drive])

    def initialize(self):

        self.drive.onChargeStation = False
        self.timer.stop()
        self.timer.reset()

    def execute(self):

        wpilib.SmartDashboard.putNumber("Yaw", self.drive.getYaw())
        wpilib.SmartDashboard.putNumber("Pitch", self.drive.getPitch())

        if self.drive.getPitch() <= 7.5 and not self.drive.onChargeStation:

            self.drive.translate(0, 0.3)
            wpilib.SmartDashboard.putString("Auto Status", "Driving to Station")

        elif self.timer.get() <= 1:

            self.drive.onChargeStation = True
            self.timer.start()

        else:

            power = (self.drive.pidController.calculate(self.drive.getPitch(), 0.0))

            wpilib.SmartDashboard.putString("Auto Status", "PID Control")

            if abs(power) <= 0.5:

                self.drive.translate(0, power)

        wpilib.SmartDashboard.putBoolean("Running", True)

    def end(self, interrupted: bool):

        self.drive.translate(0, 0)
        wpilib.SmartDashboard.putBoolean("Running", False)

    def isFinished(self):

        return False