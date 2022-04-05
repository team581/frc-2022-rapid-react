// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Slowly moves the arm to calibrate the upper and lower bounds. */
public class CalibrateArmCommand extends SequentialCommandGroup {
  /** Creates a new CalibrateArmCommand. */
  public CalibrateArmCommand() {
    addCommands();
  }
}
