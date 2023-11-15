import wpilib
from wpilib import XboxController

import commands2
import constants
from frc6343.controller.guitar.guitar import Guitar
from commands.drive_with_controller import DriveWithController
from commands.drive_with_guitar import DriveWithGuitar
from commands.charge_station import ChargeStation
# import subsystems
from subsystems.swerve_drive import SwerveDrive


class RobotContainer:
    def __init__(self) -> None:
        # init controllers
        self.driverController = XboxController(constants.kdriverControllerPort) if not constants.kUsingGuitarController else Guitar(constants.kdriverControllerPort)

        # init drive motors (may not be necessary)

        self.timer = wpilib.Timer

        # init subsystems
        self.swerveDrive = SwerveDrive()
        # auto chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to auto command chooser
        self.chargeStation = ChargeStation(self.swerveDrive)
        """
        self.simple_auto = SimpleAuto(self.drive)
        self.complex_auto = ComplexAuto(self.drive)
        #set a default option
        #add options
        #show autonomous on the driver station
        """

        self.chooser.addOption("Charge Station", self.chargeStation)

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
                DriveWithController(self.swerveDrive, lambda: self.driverController.getLeftX(),
                                    lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX()))
        

        # self.swerveDrive.setDefaultCommand(Translate(self.swerveDrive,  lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY()))
        # self.swerveDrive.setDefaultCommand(MoveInPlace(self.swerveDrive, lambda: self.driverController.getRightX()))
        # self.swerveDrive.setDefaultCommand(DriveSingleModule(self.swerveDrive,  lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY()))
        # self.swerveDrive.setDefaultCommand(TurnToSpecificPoint(self.swerveDrive,  lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY()))
        # self.swerveDrive.setDefaultCommand(Joysticks(self.swerveDrive, lambda: self.driverController.getLeftX(), lambda: self.driverController.getLeftY(), lambda: self.driverController.getRightX(), lambda: self.driverController.getRightY()))

    def configureButtonBindings(self):
        """This is where our trigger bindings for commands go"""

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()