# Team 581's FRC 2022 Rapid React robot code

[![CI](https://github.com/team581/frc-2022-rapid-react/actions/workflows/ci.yml/badge.svg)](https://github.com/team581/frc-2022-rapid-react/actions/workflows/ci.yml)

[Team 581](https://github.com/team581)'s robot code for [the FRC 2022 Rapid React game](https://youtu.be/LgniEjI9cCM).

## Features

Our robot itself is still WIP, but these are highlights of what we've accomplished on the software side so far:

- ??? ball auto
- Drivetrain
  - Mecanum drivetrain controlled with a holonomic drive controller during autonomous
  - Per-wheel closed-loop velocity control with feedforward and feedback via PID
  - Full odometry via encoders and IMU
- Vision
  - Two Limelight 2+s used for seeking cargo and aligning with the upper hub
  - Uses on-the-fly trajectory generation to plan the best route to the goal
- Intake & shooter (AKA "Swiffer Picker Upper")
  - Closed-loop velocity control for flywheel
- Arm for Swiffer Picker Upper
  - Arm position control via motion profiled PID with feedforward
- Misc
  - Mass data logging to USB + streamed to driver station via Advantage Kit

## Source code structure

Robot source code is stored within the [`src/main/java/frc/robot/` directory](./src/main/java/frc/robot).

We also have a few files within the [`src/main/java/lib/` directory](./src/main/java/lib) for generic vendor utilities (ex. a Limelight NetworkTables wrapper).

| Directory                                                                                              | Description                                                                                    |
| ------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------- |
| [`src/main/java/frc/robot/superstructure/`](./src/main/java/frc/robot/superstructure/)                 | The superstructure `Subsystem` that helps coordinate movement of the swiffer and arm together. |
| [`src/main/java/frc/robot/superstructure/swiffer/`](./src/main/java/frc/robot/superstructure/swiffer/) | The "Swiffer Picker Upper", our combined intake & shooter mechanism.                           |
| [`src/main/java/frc/robot/superstructure/arm/`](./src/main/java/frc/robot/superstructure/arm/)         | The arm mechanism for the Swiffer Picker Upper's arm.                                          |
| [`src/main/java/frc/robot/vision_cargo/`](./src/main/java/frc/robot/vision_cargo/)                     | The vision system (Limelight 2+) used for targeting cargo on the floor.                        |
| [`src/main/java/frc/robot/vision_upper/`](./src/main/java/frc/robot/vision_upper/)                     | The vision system (Limelight 2+) used for targeting the upper hub.                             |
| [`src/main/java/frc/robot/vision/`](./src/main/java/frc/robot/vision/)                                 | Code reused in the vision subsystems.                                                          |
| [`src/main/java/frc/robot/paths/`](./src/main/java/frc/robot/paths/)                                   | Tools for autonomously following paths/trajectories.                                           |
| [`src/main/java/frc/robot/drive/`](./src/main/java/frc/robot/drive/)                                   | The drivetrain.                                                                                |
| [`src/main/java/frc/robot/controller/`](./src/main/java/frc/robot/controller/)                         | Game controllers used by human drivers.                                                        |
| [`src/main/java/frc/robot/misc/`](./src/main/java/frc/robot/misc/)                                     | Miscellaneous classes that are used throughout the project and don't belong to any one group.  |

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
