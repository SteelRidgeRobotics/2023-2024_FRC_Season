from magicbot import *

from components.drivetrain import Drivetrain

class HogwaIRS(AutonomousStateMachine):

    MODE_NAME = 'HogwaIRS'
    DEFAULT = True

    drivetrain: Drivetrain

    @timed_state(duration = 5, next_state = 'forward', first = True)
    def spin(self):
        self.drivetrain.move(1, -1)

    @timed_state(duration = 10, next_state = 'stop')
    def forward(self):
        self.drivetrain.move(1, 1)

    @state
    def stop(self):
        self.drivetrain.move(0, 0)





