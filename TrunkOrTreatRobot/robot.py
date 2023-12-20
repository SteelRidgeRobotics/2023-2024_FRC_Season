import magicbot
import ctre
from ctre import TalonFX
from components.claw import Claw
from components.arm import Arm
from components.tank import Tank4MDrive
from constants import *
import wpilib

class TrunkOrTreatRobot(magicbot.MagicRobot):
    claw: Claw
    arm: Arm
    #tank: Tank4MDrive

    def createObjects(self):
        self.armMotor = ctre.TalonSRX(MOTORID) #Arm motor
        self.grabberSolenoid = wpilib.DoubleSolenoid(CLAWSOLENOIDID, wpilib.PneumaticsModuleType.CTREPCM, SOLENOIDFORWARDCHANNEL, SOLENOIDREVERSECHANNEL)
        self.functionsController = wpilib.XboxController(FUNCTIONSPORT)

        self.leftFollower = TalonFX(kleftFollowerId)
        self.rightFollower = TalonFX(krightFollowerId)
        self.leftMaster = TalonFX(kleftMasterId)
        self.rightMaster = TalonFX(krightMasterId)

    def teleopPeriodic(self):
        self.claw.move(self.functionsController.getAButton(), self.functionsController.getBButton())
        self.arm.move(self.functionsController.getRightTriggerAxis() - self.functionsController.getLeftTriggerAxis())
        #self.tank.setMags(-self.functionsController.getLeftY(), -self.functionsController.getLeftY())

if __name__ == "__main__":

    wpilib.run(TrunkOrTreatRobot)