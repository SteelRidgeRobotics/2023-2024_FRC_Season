import wpilib

class Claw:

    grabberSolenoid: wpilib.DoubleSolenoid

    closed = 0

    def move(self, guitarGreen, guitarRed):

        if guitarGreen:
            self.closed = 1
        elif guitarRed:
            self.closed = -1
        else:
            self.closed = 0
    
    def execute(self):

        if self.closed == 1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.closed == -1:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        else:
            self.grabberSolenoid.set(wpilib.DoubleSolenoid.Value.kOff)

        if self.grabberSolenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("Grabber Status", "Forward")
        elif self.grabberSolenoid.get() == wpilib.DoubleSolenoid.Value.kReverse:
            wpilib.SmartDashboard.putString("Grabber Status", "Reverse")
        else:
            wpilib.SmartDashboard.putString("Grabber Status", "Off")
    