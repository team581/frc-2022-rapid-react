# FRC 2022 Rapid React

[![CI](https://github.com/team581/frc-2022-rapid-react/actions/workflows/ci.yml/badge.svg)](https://github.com/team581/frc-2022-rapid-react/actions/workflows/ci.yml)

[Team 581](https://github.com/team581)'s robot code for [the FRC 2022 Rapid React game](https://youtu.be/LgniEjI9cCM).

## Features

Our robot itself is still WIP, but these are highlights of what we've accomplished on the software side so far:

- ??? ball auto
- Drivetrain
  - Mecanum drivetrain controlled with a holonomic drive controller during autonomous
  - Per-wheel closed-loop velocity control via PID & feedforward
- Vision
  - Two Limelight 2+s used for seeking cargo and aligning with the upper hub
  - Uses on-the-fly trajectory generation to plan the best route to the goal

## Source code structure

Robot source code is stored within the [`src/main/java/frc/robot/` directory](./src/main/java/frc/robot).

We also have a few files within the [`src/main/java/lib/` directory](./src/main/java/lib) for generic vendor utilities (ex. a Limelight NetworkTables wrapper).

| Directory                                                                      | Description                                                                                  |
| ------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------- |
| [`src/main/java/frc/robot/commands/`](./src/main/java/frc/robot/commands/)     | Commands & command groups that can be triggered by a driver or during the autonomous period. |
| [`src/main/java/frc/robot/paths/`](./src/main/java/frc/robot/paths/)           | Exposes human-created paths/trajectories as code.                                            |
| [`src/main/java/frc/robot/subsystems/`](./src/main/java/frc/robot/subsystems/) | Subsystems that control our robot's mechanisms, ie. """business logic""".                    |
| [`src/main/java/frc/robot/util/`](./src/main/java/frc/robot/util/)             | Misc utilities that are used throughout the project.                                         |
| [`src/main/java/frc/robot/vision/`](./src/main/java/frc/robot/vision/)         | Stores code related to the vision system, mostly just vision targets.                        |

## Limelight configuration files

See [`limelight/`](./limelight/README.md).

## PathPlanner trajectories

PathPlanner trajectories are stored within the [`src/main/deploy/pathplanner/` directory](./src/main/deploy/pathplanner/).

## WPILib SysId

Assorted WPILib SysId files are stored in the top-level project directory and the [`.SysId/` directory](./.SysId/).

| File                           | Description                             |
| ------------------------------ | --------------------------------------- |
| [`config.json`](./config.json) | The main SysId configuration file.      |
| `sys_id_data*.json`            | Recorded SysId data files for analysis. |
