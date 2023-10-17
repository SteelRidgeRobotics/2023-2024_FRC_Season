import constants
import ctre
import wpilib
import magicbot
from guitar.guitar import Guitar
from robot import Guitar

class Claw:

    grabberSolenoid: wpilib.DoubleSolenoid

    closed = 0

    def move(self, greenButton, redButton):

        self.green = greenButton
        self.red = redButton

        if self.green:
            self.closed = 1
        elif self.red:
            self.closed = -1
        else:
            self.closed = 0
    
    def execute(self):
        if self.closed==1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.closed==-1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        else:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kOff)