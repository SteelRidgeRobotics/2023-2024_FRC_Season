import wpilib
import magicbot
import ctre
from components.claw import Claw
from components.arm import Arm
import constants
from guitar.guitar import Guitar


class TrunkOrTreatRobot(magicbot.MagicRobot):

    claw: Claw
    arm: Arm

    def createObjects(self):

        self.armMotor = ctre.TalonSRX(constants.MOTORID) #Arm motor
        self.grabberSolenoid = wpilib.DoubleSolenoid(constants.CLAWSOLENOIDID, wpilib.PneumaticsModuleType.CTREPCM, constants.SOLENOIDFORWARDCHANNEL, constants.SOLENOIDREVERSECHANNEL)
        self.functionsController = Guitar(constants.FUNCTIONSPORT)

    def teleopPeriodic(self):

        self.claw.move(self.functionsController.getGreenButton(), self.functionsController.getRedButton())
        self.arm.move(self.functionsController.getJoystickY())

if __name__ == "__main__":

    wpilib.run(TrunkOrTreatRobot)