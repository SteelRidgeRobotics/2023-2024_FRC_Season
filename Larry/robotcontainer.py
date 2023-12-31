import wpilib
from wpilib import XboxController

import commands2
import constants
from frc6343.controller.guitar.guitar import Guitar
from commands.drive_with_controller import DriveControllerDefault, DriveControllerDefaultSlow, DriveControllerWyatt
from commands.drive_with_guitar import DriveWithGuitar
from commands.charge_station import ChargeStation
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
        self.profileChooser.setDefaultOption("Default", DriveControllerDefault(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper()))
        self.profileChooser.addOption("Default Slow", DriveControllerDefaultSlow(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper()))
        self.profileChooser.addOption("Wyatt", DriveControllerWyatt(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getLeftBumper(), lambda: self.driverController.getRightBumper(), lambda: self.driverController.getLeftTriggerAxis(), lambda: self.driverController.getRightTriggerAxis()))

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

    def configureButtonBindings(self):
        """This is where our trigger bindings for commands go"""

    def getAutonomousCommand(self) -> commands2.Command:
        return self.autoChooser.getSelected()
    
    def getDrivingMode(self) -> commands2.Command:
        return self.profileChooser.getSelected()
    
    def getSwerveDrive(self) -> commands2.SubsystemBase:
        return self.swerveDrive