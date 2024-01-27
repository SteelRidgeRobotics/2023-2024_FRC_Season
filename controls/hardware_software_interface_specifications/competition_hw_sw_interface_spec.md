# Controls Software Specification

Robot Name: TBD

## Description

This document describes the interface between the hardware and software of the competition robot for the 20024 Crescendo FRC Season.
The robot consists of a swerve drive chassis, an elevator mechanism, and an intake.
The swerve drive uses the Swerve Drive Specialties MK4i module arranged on a square chassis.
The elevator mechanism, known as the lift, consists of two telescoping lifts that support the intake mechansim.
The lift also includes a pair of hooks that can allow the robot to climb the chain in the end game.
The intake uses horizontal rollers that grab and hold on to the game piece.
The intake pivots between a retracted position inside the frame perimeter and a deployed position that reaches over the bumper.

## Robot Controllers

This robot uses a single RoboRIO 2 as the only robot controller.
The robot code is implemented in Python using the 2024 RobotPy framework.
Major software versions are shown in the following table.  

| Software Component      | Version    |
|-------------------------|------------|
| RobotRIO Image Version: | 2024 v2.1  |
| Python                  | 3.11.7     |
| OpenMesh Radio Firmware | TBD        |
| _Python Packages_       |            |
| robotpy                 | 2024.2.2.1 |
| robotpy-commands-v2     | 2024.2.1   |
| frc6343                 | 0.2        |


## Motor Controllers

| Motor          | Controller   | FW     | Motor    | CAN Addr | PDP Port | Breaker |
|----------------|--------------|--------|----------|:--------:|:--------:|---------|
| Swerve 0 Drive | Talon FX     | 2023.3 | Falcon   |    1     |    0     | 40 A    |
| Swerve 0 Steer | Talon FX     |        | Falcon   |    2     |    0     | 30 A    |
| Swerve 1 Drive | Talon FX     |        | Falcon   |    3     |    0     | 40 A    |
| Swerve 1 Steer | Talon FX     |        | Falcon   |    4     |    0     | 30 A    |
| Swerve 2 Drive | Talon FX     |        | Falcon   |    5     |    0     | 40 A    |
| Swerve 2 Steer | Talon FX     |        | Falcon   |    6     |    0     | 30 A    |
| Swerve 3 Drive | Talon FX     |        | Falcon   |    7     |    0     | 40 A    |
| Swerve 3 Steer | Talon FX     |        | Falcon   |    8     |    0     | 30 A    |
| Lift           | Talon FX     |        | Falcon   |    9     |    0     | 40 A    |
| Intake Pivot   | Talon FX     |        | Falcon   |    10    |    0     | 40 A    |
| Intake Feeder  | Talon FX     |        | Falcon   |    11    |    0     | 30 A    |

## OpenMesh Radio

The OpenMesh radio is connected directly to the RoboRIO 2 through an Ethernet cable. The radio is powered from the
VRM's 12V, 2A output which is connected to the DC barrel connector on the radio.

## Crimp Inspection Log

The crimp inspections on this robot are undocumented.
