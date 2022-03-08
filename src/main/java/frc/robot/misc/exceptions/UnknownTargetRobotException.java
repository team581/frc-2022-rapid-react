// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.exceptions;

import frc.robot.Constants;

public class UnknownTargetRobotException extends IllegalStateException {
  public UnknownTargetRobotException() {
    super("Unknown target robot: " + Constants.getRobot().toString());
  }
}
