# Hardware Software Interface Specification

Robot Name: _TBD_

<!-- TOC -->
* [Hardware Software Interface Specification](#hardware-software-interface-specification)
  * [Software Repository](#software-repository)
  * [Description](#description)
    * [Swerve Drive Mechanism](#swerve-drive-mechanism)
    * [Game Piece Handling Mechanisms](#game-piece-handling-mechanisms)
      * [Lift](#lift)
      * [Intake Pivot](#intake-pivot)
      * [Intake Feeder](#intake-feeder)
  * [Program Priorities](#program-priorities)
  * [Motion Control Rules](#motion-control-rules)
    * [Lift and Intake Rules](#lift-and-intake-rules)
  * [Robot Controllers](#robot-controllers)
  * [Motor Controllers](#motor-controllers)
  * [Sensors](#sensors)
  * [OpenMesh Radio](#openmesh-radio)
  * [Useful Links](#useful-links)
  * [Crimp Inspection Log](#crimp-inspection-log)
<!-- TOC -->

## Software Repository
The robot source code can be found in this repository: [Competition Robot Code](https://github.com/SteelRidgeRobotics/2024DumpsterFire)


## Description

This document describes the interface between the hardware and software of the competition robot for the 2024 Crescendo FRC Season.
The robot consists of a swerve drive chassis, an elevator mechanism, and an intake.
The swerve drive uses the Swerve Drive Specialties MK4i module arranged on a square chassis.
The elevator mechanism, known as the lift, consists of two telescoping lifts that support the Touch It, Own It (TITO) intake mechanism.
The lift also includes a pair of hooks that can allow the robot to climb the chain in the end game.
The intake uses horizontal rollers that grab and hold on to the game piece.
The intake pivots between a retracted position inside the frame perimeter and a deployed position that reaches over the bumper.

### Swerve Drive Mechanism

The swerve drive utilizes the Swerve Drive Specialties MK4i module.
The steering gear ratio of the MK4i is 150/7:1.
The drive gear ratio of the MK4i is 27/4:1.
This is the V2, or medium speed, version of the module.
Each steering mechanism includes a CANcoder absolute position encoder.

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

The intake rotates around a pivot at the top of the lift.
The pivot is driven by a single motor and gearbox.
The pivot will rotate the intake to a predetermined rotation position along its range of motion.
The retracted, or parked, position is at one extreme of the pivot's range of motion where the intake is fully inside the robot's frame perimeter.
The extended, or deployed, position is at the other extreme of the pivot's range of motion.
The extended position is used when capturing game pieces directly off the floor.

In the preferred control implementation, the control algorithm can rotate the pivot to an arbitrary angle and hold the intake in that position for the duration of a match. 
The specific implementation is _TBD_, but something like Phoenix 6 Motion Magic or the WPI ProfiledPID Controller would likely work well.

The home position of the pivot is the fully retracted position. 
The robot will start the match in the fully retracted position, so the robot code can set this as the home position during robot initialization.
An incremental quadrature encoder will be used to track the position of the pivot.
A in match re-homing algorithm is not included in the design, so the motor current must be limited to prevent motor controller brown out / black out.

An alternative position sensor approaches include using an absolute position sensor.
An alternative approach to homing the pivot would through the use of a micro-switch or a magnet and Hall effect sensor. 

_Note: At the time of this writing, the specific motor and gearbox gear ratio is TBD.
However, the motor will likely be a Falcon 500 and the gear ratio will likely be 48:1._

#### Intake Feeder

The intake feeder will be driven by a single motor.
The use of an encoder for speed control is _TBD_.
A beam break sensor will detect when the intake is populated with a game piece.

_Note: At the time of this writing, the specific motor and gearbox gear ratio is TBD._

## Program Priorities

The various robot capabilities will be developed using the following priority list.
This list should be used to guide trade off decisions to ensure that lower priority subsystems are not preventing progress on higher priority subsystems.
1. Drivetrain, Chassis & Bumpers
2. Touch It, Own It (TIOI) Intake
3. Amplifier Lift
4. Climber
5. Other

"Other" development efforts include scoring in the TRAP and utilizing vision to improve teleop performance.

## Motion Control Rules

_Note: The high level motion control rules for the robot are under development._

The high level motion control rules describe the allowed interactions between the various robot subsystems.

### Lift and Intake Rules
These rule govern the interaction between the lift and intake mechanisms.
1. Only move one mechanism at a time.
2. Only move the intake pivot when the lift is in the low position.
3. Only raise the lift when the intake is not in the retracted position.
4. Automatically retract the intake when a game piece is captured by the intake.
5. Run the feeder in the in capture direction when the intake is empty and not in the retracted position.

## Robot Controllers

The robot implements a CTRE style robot control system.
A generic CTRE control system is shown below.

![](https://github.com/SteelRidgeRobotics/2023-2024_FRC_Season/blob/74a8d3010bef6135c66bc175156146020acc0bac/controls/hardware_software_interface_specifications/frc-control-system-layout.png)

This robot uses a single RoboRIO 2 as the only robot controller.
The robot code is implemented in Python using the 2024 RobotPy framework.
Major software versions are shown in the following table.
Motor controller and sensor software versions are listed elsewhere in this document.

| Software Component        | Version       |
|---------------------------|---------------|
| RobotRIO Image            | 2024 v2.1     |
| Python                    | 3.11.7        |
| Power Distribution Module | 1.4           |
| OpenMesh Radio            | _TBD_         |
| CTRE Motion Control       | Phoenix 6 Pro |

| Python Packages         | Version    |
|-------------------------|------------|
| robotpy                 | 2024.2.1.1 |
| frc6343                 | 0.2        |
 
Installed Robotpy Modules: commands2, navx, pathplannerlib, phoenix6

## Motor Controllers

The following table lists all the motor controllers used in the robot.

| Function          | Controller  | FW          | Motor    | CAN Addr | PDP Port | Breaker |
|-------------------|-------------|-------------|----------|:--------:|:--------:|:-------:|
| Drive Front Left  | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    13    |  40 A   |
| Drive Rear Left   | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    14    |  40 A   |
| Drive Front Right | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    12    |  40 A   |
| Drive Rear Right  | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    15    |  40 A   |
| Steer Front Left  | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    10    |  30 A   |
| Steer Rear Left   | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    9     |  30 A   |
| Steer Front Right | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    11    |  30 A   |
| Steer Rear Right  | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |    8     |  30 A   |
| Lift Right        | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |  _TBD_   |  40 A   |
| Lift Left         | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |  _TBD_   |  40 A   |
| Intake Pivot      | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |  _TBD_   |  40 A   |
| Intake Feeder     | Talon FX    | 2024.1.0.0  | Falcon   |  _TBD_   |  _TBD_   |  40 A   |


## Discrete Sensors

The following table lists all the sensors used in the robot.

| Function                 | Sensor              |     FW     | Associated Motor      | CAN Addr | PDP / VRM     |
|--------------------------|---------------------|:----------:|-----------------------|:--------:|---------------|
| Steer Front Left Swerve  | CANcoder            | 2024.1.0.0 | Front Left Direction  |    10    | VRM 12V/500mA |
| Steer Rear Left Swerve   | CANcoder            | 2024.1.0.0 | Rear Left Direction   |    11    | VRM 12V/500mA |
| Steer Front Right Swerve | CANcoder            | 2024.1.0.0 | Front Right Direction |    12    | VRM 12V/500mA |
| Steer Rear Right Swerve  | CANcoder            | 2024.1.0.0 | Rear Right Direction  |    13    | VRM 12V/500mA |
| Intake Feeder Beam Break | Adafruit Beam Break |    ---     | Intake Feeder         |   ---    | VRM 5V/500mA  |

Notes:
1.  The VRM is powered through a 20 A breaker at **PDP Port 4**.
2.  Integrated sensors built into Falcon 500 motors are not included in this list.

## Ethernet

A Brainbox SW-005 100Mbps Ethernet switch connects the roboRIO to the OpenMesh radio and the Limelight 3 vision module.

| Device          |     PDP Port     | Breaker/ Fuse |
|-----------------|:----------------:|:-------------:|
| Ethernet Switch |        5         |     20 A      |
| Limelight 3     |      _TBD_       |     20 A      |
| OpenMesh Radio  | Vbat VRM PCM PWR |     20 A      |

### OpenMesh Radio

The OpenMesh radio is connected directly to the Radio Power Module through an Ethernet cable.
Similarly, the Radio Power Module is connected directly to Ethernet Switch.
The PDP's "Vbat VRM PCM PWR" output is connected to a Radio Power Module (RPM) with provides power to the radio through the Ethernet cable.

### Limelight 3

The Limelight vision module os connected directly to Ethernet Switch through an Ethernet cable.

## Useful Links

[2024 Crescendo Game Resources](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)
