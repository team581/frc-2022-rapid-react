// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimelightSubsystem;

/** The 2020 loading bay vision target. Exclusively used for debugging purposes. */
public class LoadingBay extends LimelightVisionTarget {
  public LoadingBay(LimelightSubsystem limelight) {
    super(limelight, Units.inchesToMeters(11));
  }

  @Override
  public void onSelected() {
    super.onSelected();

    limelightSubsystem.limelight.setPipeline(1);
  }
}
