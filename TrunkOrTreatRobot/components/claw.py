import constants
import ctre
import wpilib
import magicbot
from guitar.guitar import Guitar

class Claw:

    grabberSolenoid: wpilib.DoubleSolenoid

    closed = 0

    def move(self, guitar: Guitar):

        if guitar.getGreenButtonPressed():
            self.closed = 1
        elif guitar.getRedButtonPressed():
            self.closed = -1
    
    def execute(self):
        if self.closed==1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.closed==-1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        wpilib.SmartDashboard.putNumber("Claw Closed?", self.closed)

    