import constants
import components.drivetrain
import magicbot
import ctre
import wpilib

class HarryPottah(magicbot.MagicRobot):

    drivetrain: components.drivetrain.Drivetrain

    def createObjects(self):

        self.FLMotor = ctre.TalonFX(constants.FLMOTORID)
        self.BLMotor = ctre.TalonFX(constants.BLMOTORID)
        self.FRMotor = ctre.TalonFX(constants.FRMOTORID)
        self.BRMotor = ctre.TalonFX(constants.BRMOTORID)

        self.driverController = wpilib.XboxController(constants.DRIVERPORT)

    def teleopPeriodic(self):

        self.drivetrain.move(self.driverController.getLeftY(), self.driverController.getRightY())

if __name__ == "__main__":

    wpilib.run(HarryPottah)