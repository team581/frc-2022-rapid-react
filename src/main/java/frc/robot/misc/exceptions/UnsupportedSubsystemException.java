// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.exceptions;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.misc.SubsystemIO;

public class UnsupportedSubsystemException extends IllegalStateException {
  public UnsupportedSubsystemException(Class<?> subsystem) {
    this(subsystem.getName(), "subsystem");
  }

  public UnsupportedSubsystemException(SubsystemBase subsystem) {
    this(subsystem.getName(), "subsystem");
  }

  public UnsupportedSubsystemException(SubsystemIO io) {
    this(io.getClass().getName(), "subsystem IO");
  }

  private UnsupportedSubsystemException(String name, String kind) {
    super(
        String.format(
            "The configured %s robot doesn't support the %s %s",
            Constants.getRobot().toString(), name, kind));
  }
}
