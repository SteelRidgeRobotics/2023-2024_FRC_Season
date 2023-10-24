import wpilib

class SolenoidBot(wpilib.TimedRobot):

    def robotInit(self):

        doubleSolenoid = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, 0, 1)

if __name__ == "__main__":

    wpilib.run(SolenoidBot)