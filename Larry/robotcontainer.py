import wpilib
from wpilib import XboxController

import commands2
import constants
from frc6343.controller.guitar.guitar import Guitar
from commands.drive_with_controller import DriveWithController
from commands.drive_with_guitar import DriveWithGuitar
from commands.charge_station import ChargeStation
from commands.set_driver_profile import DriverProfiles, SetDriverProfile
# import subsystems
from subsystems.swerve_drive import SwerveDrive

class RobotContainer:
    def __init__(self) -> None:
        # init controllers
        self.driverController = XboxController(constants.kdriverControllerPort) if not constants.kUsingGuitarController else Guitar(constants.kdriverControllerPort)

        self.timer = wpilib.Timer

        # init subsystems
        self.swerveDrive = SwerveDrive()

        # auto chooser
        self.autoChooser = wpilib.SendableChooser()

        # Add commands to auto command chooser
        self.autoChooser.addOption("Charge Station", ChargeStation(self.swerveDrive))

        wpilib.SmartDashboard.putData("Auto", self.autoChooser)

        # profile chooser
        self.profileChooser = wpilib.SendableChooser()
        self.profileChooser.setDefaultOption("Default", SetDriverProfile(self.swerveDrive, DriverProfiles.DEFAULT))
        self.profileChooser.addOption("Default Slow | Bumper Speedup", SetDriverProfile(self.swerveDrive, DriverProfiles.DEFAULT_SLOW_BUMPER_SPEEDUP))

        wpilib.SmartDashboard.putData("Profile", self.profileChooser)

        self.configureButtonBindings()

        if constants.kUsingGuitarController:
            self.swerveDrive.setDefaultCommand(
                DriveWithGuitar(self.swerveDrive,
                                lambda: self.driverController.getGreenButtonPressed(),
                                lambda: self.driverController.getRedButtonPressed(),
                                lambda: self.driverController.getYellowButtonPressed(),
                                lambda: self.driverController.getBlueButtonPressed(),
                                lambda: self.driverController.getStrumBarDownPressed(),
                                lambda: self.driverController.getStrumBarUpPressed()
                )
            )
        else:
            self.swerveDrive.setDefaultCommand(
                DriveWithController(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), 
                                    lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper()))

    def configureButtonBindings(self):
        """This is where our trigger bindings for commands go"""

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
    
    def getDriverProfile(self) -> commands2.Command:
        return self.profileChooser.getSelected()