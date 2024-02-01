from enum import Enum
from wpiutil import *
from phoenix6.signals import InvertedValue, SensorDirectionValue
from wpimath.units import *
#from Swervemodule import *
#from SwerveModuleConstants import *


class InvertedValue(Enum):
    Clockwise_Positive = 1
    CounterClockwise_Positive = 2

class SensorDirectionValue(Enum):
    CounterClockwise_Positive = 1
    Clockwise_Positive = 2

class COTSTalonFXSwerveConstants:
    def __init__(self, wheel_diameter:float, angle_gear_ratio:float, drive_gear_ratio:float, angle_KP:float, angle_KI:float, angle_KD:float, drive_motor_invert:InvertedValue, angle_motor_invert:InvertedValue, cancoder_invert:SensorDirectionValue):
        self.wheel_diameter = wheel_diameter
        self.wheel_circumference = wheel_diameter * math.pi
        self.angle_gear_ratio = angle_gear_ratio
        self.drive_gear_ratio = drive_gear_ratio
        self.angle_KP = angle_KP
        self.angle_KI = angle_KI
        self.angle_KD = angle_KD
        self.drive_motor_invert = drive_motor_invert
        self.angle_motor_invert = angle_motor_invert
        self.cancoder_invert = cancoder_invert

class COTSTalonFXSwerveConstants_WCP:
    class SwerveXStandard:
        class driveRatios:
            X1_10 = 7.85
            X1_11 = 7.13
            X1_12 = 6.54
            X2_10 = 6.56
            X2_11 = 5.96
            X2_12 = 5.46
            X3_12 = 5.14
            X3_13 = 4.75
            X3_14 = 4.41

        @staticmethod
        def Falcon500(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (396.0 / 35.0) / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

        @staticmethod
        def KrakenX60(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (396.0 / 35.0) / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

    class SwerveXFlipped:
        class driveRatios:
            X1_10 = 8.10
            X1_11 = 7.36
            X1_12 = 6.75
            X2_10 = 6.72
            X2_11 = 6.11
            X2_12 = 5.60
            X3_10 = 5.51
            X3_11 = 5.01
            X3_12 = 4.59

        @staticmethod
        def Falcon500(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (468.0 / 35.0) / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

        @staticmethod
        def KrakenX60(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (468.0 / 35.0) / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

class COTSTalonFXSwerveConstants_SDS:
    class MK3:
        class driveRatios:
            Standard = 8.16
            Fast = 6.86

        @staticmethod
        def Falcon500(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = 12.8 / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.CounterClockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

        @staticmethod
        def KrakenX60(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = 12.8 / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.CounterClockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

    class MK4:
        class driveRatios:
            L1 = 8.14
            L2 = 6.75
            L3 = 6.12
            L4 = 5.14

        @staticmethod
        def Falcon500(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = 12.8 / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.CounterClockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

        @staticmethod
        def KrakenX60(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = 12.8 / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.CounterClockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

    class MK4i:
        @staticmethod
        def Falcon500(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (150.0 / 7.0) / 1.0
            angle_KP = 100.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)

        @staticmethod
        def KrakenX60(drive_gear_ratio):
            wheel_diameter = 0.1016
            angle_gear_ratio = (150.0 / 7.0) / 1.0
            angle_KP = 1.0
            angle_KI = 0.0
            angle_KD = 0.0
            drive_motor_invert = InvertedValue.CounterClockwise_Positive
            angle_motor_invert = InvertedValue.Clockwise_Positive
            cancoder_invert = SensorDirectionValue.CounterClockwise_Positive
            return COTSTalonFXSwerveConstants(wheel_diameter, angle_gear_ratio, drive_gear_ratio, angle_KP, angle_KI, angle_KD, drive_motor_invert, angle_motor_invert, cancoder_invert)
    
        class driveRatios:
            L1 = 8.14
            L2 = 6.75
            L3 = 6.12