from wpimath import *

class Conversions:
    
    @staticmethod
    def RPS_to_MPS(wheel_RPS, circumference):
        wheel_MPS = wheel_RPS * circumference
        return wheel_MPS

    @staticmethod
    def MPS_to_RPS(wheel_MPS, circumference):
        wheel_RPS = wheel_MPS / circumference
        return wheel_RPS

    @staticmethod
    def rotations_to_meters(wheel_rotations, circumference):
        wheel_meters = wheel_rotations * circumference
        return wheel_meters

    @staticmethod
    def meters_to_rotations(wheel_meters, circumference):
        wheel_rotations = wheel_meters / circumference
        return wheel_rotations