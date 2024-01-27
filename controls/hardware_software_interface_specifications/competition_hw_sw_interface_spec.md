# Hardware Software Specification

Robot Name: TBD

## Description

This document describes the interface between the hardware and software of the competition robot for the 20024 Crescendo FRC Season.
The robot consists of a swerve drive chassis, an elevator mechanism, and an intake.
The swerve drive uses the Swerve Drive Specialties MK4i module arranged on a square chassis.
The elevator mechanism, known as the lift, consists of two telescoping lifts that support the intake mechansim.
The lift also includes a pair of hooks that can allow the robot to climb the chain in the end game.
The intake uses horizontal rollers that grab and hold on to the game piece.
The intake pivots between a retracted position inside the frame perimeter and a deployed position that reaches over the bumper.

### Swerve Drive Mechanism

The swerve drive utilizes the Swerve Drive Specialties MK4i module.
The steering gear ratio of the MK4i is 150/7:1.
The drive gear ratio of the MK4i is 27/4:1.
This is the V2, or medium speed, version of the module.

[SDS MK4i Webpage](https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081)

### Game Piece Handling Mechanisms

The game piece handling portion of the robot consists of the lift and the intake.
Game piece handling is managed through three degrees of freedom (DoF).

#### Lift

The lift is an elevator system that supports the intake mechanism and serves as the climber mechanism.
The lift consists of a pair of telescoping booms that extend vertically.
The intake is suspended between the booms.
Each boom hosts a hook that can snag the chain on the stage so the robot can climb at the end of the match.

Each boom is spring-loaded.
The springs extend each boom to its maximum height with approximately 20 lbs (89 N) of force.
Each boom is retracted using a 3 mm Dyneema (or equivalent) rope attached to a spindle.
The two spindles, one for each boom, are mechanically linked together and drive by a single motor.
The total extension force experienced by the motor at the spindles is approximately 180 N.

The method of finding the home position of the lift is _TBD_.
However, the preferred alternative is to retract the lift while monitoring the motor current.
When the motor current exceeds a threshold for a specified period of time, the left will be assumed to be in the fully retracted position.
This will be the home position of the lift.
The motor controller will need to set a current limit to prevent this homing algorithm from tripping the circuit breaker.

An alternative approach to determining the home position fo the lift through the use of a magnet and Hall effect sensor.
Also, a mechanical micro switch can also be used.
The micro switch, magnet and Hall effect sensor are not currently planned to be part of the mechanical design.

_Note: At the time of this writing, the specific motor and gearbox gear ratio is TBD.
However, the motor will likely be a Falcon 500._

#### Intake Pivot

_Note: At the time of this writing, the specific motor and gearbox gear ratio is TBD.
However, the motor will likely be a Falcon 500 and the gear ratio will likely be 48:1._

#### Intake Feeder

_Note: At the time of this writing, the specific motor and gearbox gear ratio is TBD._

## Robot Controllers

This robot uses a single RoboRIO 2 as the only robot controller.
The robot code is implemented in Python using the 2024 RobotPy framework.
Major software versions are shown in the following table.  

| Software Component      | Version    |
|-------------------------|------------|
| RobotRIO Image Version: | 2024 v2.1  |
| Python                  | 3.11.7     |
| OpenMesh Radio Firmware | TBD        |

| Python Packages         | Version    |
|-------------------------|------------|
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
