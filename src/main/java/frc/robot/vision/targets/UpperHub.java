// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimelightSubsystem;

/** The upper hub ring vision target. */
public class UpperHub extends LimelightVisionTarget {
  public UpperHub(LimelightSubsystem limelight) {
    super(limelight, Units.feetToMeters(8) + Units.inchesToMeters(5.0 + (5.0 / 8.0)));
  }

  @Override
  public void onSelected() {
    super.onSelected();

    limelightSubsystem.limelight.setPipeline(0);
  }
}
