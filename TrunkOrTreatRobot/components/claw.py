import constants
import ctre
import wpilib
import magicbot
from guitar.guitar import Guitar
from robot import Guitar

class Claw(StateMachine):

    grabberSolenoid: wpilib.DoubleSolenoid

if Guitar.getGreenButtonPressed(): #Green button opens claw
    pass

if Guitar.getRedButtonPressed(): #Red button closes claw
    pass

if Guitar.getBlueButtonPressed(): #Blue button moves arm up
    pass

if Guitar.getOrangeButtonPressed(): #Orange button moves arm down
    pass