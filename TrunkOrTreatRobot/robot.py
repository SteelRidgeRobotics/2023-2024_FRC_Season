import wpilib
import magicbot
import ctre
import constants


class TrunkOrTreatRobot(magicbot.MagicRobot):

    def createObjects(self):

        self.armMotor = ctre.TalonFX(constants.MOTORID) #Arm motor
        self.grabberSolenoid = wpilib.DoubleSolenoid(constants.SOLENOIDID1) #Not sure what ID is for solenoids. Our guess is that it's wpilib.solenoid
        self.wristSolenoid = wpilib.DoubleSolenoid(constants.SOLENOIDID2) #Same issue here

        self.driverController = wpilib.XboxController(constants.DRIVERPORT) #We might have to change the controller if we're using the guitar hero

    def teleopPeriodic(self):

        pass #Code needed here

if __name__ == "__main__":

    wpilib.run(TrunkOrTreatRobot)