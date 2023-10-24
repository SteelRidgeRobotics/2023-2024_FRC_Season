import wpilib
import magicbot
import ctre
from components.claw import Claw
import constants
from guitar.guitar import Guitar


class TrunkOrTreatRobot(magicbot.MagicRobot):

    claw: Claw

    def createObjects(self):

        self.armMotor = ctre.TalonFX(constants.MOTORID) #Arm motor
        self.grabberSolenoid = wpilib.DoubleSolenoid(constants.CLAWSOLENOIDID, wpilib.PneumaticsModuleType.CTREPCM, constants.SOLENOIDFORWARDCHANNEL, constants.SOLENOIDREVERSECHANNEL) #Not sure what ID is for solenoids. Our guess is that it's wpilib.solenoid
        self.functionsController = Guitar(constants.FUNCTIONSPORT)

    def teleopPeriodic(self):

        self.claw.move(self.functionsController)

        wpilib.SmartDashboard.putString("Test 1", "Hello world")
        wpilib.SmartDashboard.putBoolean("e", self.functionsController.getGreenButton())

if __name__ == "__main__":

    wpilib.run(TrunkOrTreatRobot)