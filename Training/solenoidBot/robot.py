import wpilib
import ctre

class SolenoidBot(wpilib.TimedRobot):

    def robotInit(self):

        solenoid = wpilib.DoubleSolenoid()

if __name__ == "__main__":

    wpilib.run(SolenoidBot)