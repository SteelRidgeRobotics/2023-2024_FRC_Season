# Controls Software Specification

Robot Name: Larry

## Description

This robot is the test platform for developing the team's first swerve drive system.

## Robot Controllers

This robot uses a single RoboRIO 2 as the only robot controller.

## Motor Controllers

| Motor              | Controller   | FW     | Motor    |  CAN Addr  | PDP Port | Breaker |
|--------------------|--------------|--------|----------|:----------:|:--------:|---------|
| Swerve 0 Drive     | Talon FX     | 2023.3 | Falcon   |     1      |    0     | 40 A    |
| Swerve 0 Steer     | Talon FX     |        | Falcon   |     2      |    0     | 30 A    |
| Swerve 1 Drive     | Talon FX     |        | Falcon   |     3      |    0     | 40 A    |
| Swerve 1 Steer     | Talon FX     |        | Falcon   |     4      |    0     | 30 A    |
| Swerve 2 Drive     | Talon FX     |        | Falcon   |     5      |    0     | 40 A    |
| Swerve 2 Steer     | Talon FX     |        | Falcon   |     6      |    0     | 30 A    |
| Swerve 3 Drive     | Talon FX     |        | Falcon   |     7      |    0     | 40 A    |
| Swerve 3 Steer     | Talon FX     |        | Falcon   |     8      |    0     | 30 A    |

## OpenMesh Radio

The OpenMesh radio is connected directly to the RoboRIO 2 through an Ethernet cable. The radio is powered from the
VRM's 12V, 2A output which is connected to the DC barrel connector on the radio.
