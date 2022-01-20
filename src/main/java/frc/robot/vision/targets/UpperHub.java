// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.LimelightSubsystem;

/** The upper hub ring vision target. */
public class UpperHub extends LimelightVisionTarget {
  public UpperHub(LimelightSubsystem limelight) {
    super(limelight);
  }

  @Override
  public void onSelected() {
    super.onSelected();

    limelight.setPipeline(0);
  }
}
