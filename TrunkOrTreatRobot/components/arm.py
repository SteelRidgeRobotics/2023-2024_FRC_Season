import constants
import ctre
import wpilib
from magicbot import StateMachine


class Arm:

    armMotor: ctre.TalonSRX

    def setup(self):

        self.armMotor.setNeutralMode(ctre.NeutralMode.Brake)

    def move(self, leftJoyY):

        self.left = leftJoyY


        if abs(self.left) <= constants.DEADBAND:

            self.left = 0



    def execute(self):

        self.armMotor.set(ctre.TalonSRXControlMode.PercentOutput, -self.left)





    

    
