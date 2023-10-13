import constants
import ctre
import wpilib
import magicbot

class Claw(StateMachine):

    grabberSolenoid: wpilib.DoubleSolenoid

    
