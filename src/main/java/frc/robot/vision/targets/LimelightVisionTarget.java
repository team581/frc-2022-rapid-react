// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.limelight.Limelight;
import frc.robot.subsystems.LimelightSubsystem;

/** A vision target for the Limelight. */
public abstract class LimelightVisionTarget extends VisionTarget {
  protected final Limelight limelight;

  protected LimelightVisionTarget(LimelightSubsystem limelightSubsystem) {
    this.limelight = limelightSubsystem.limelight;
  }

  @Override
  public void onSelected() {
    limelight.setLEDMode(Limelight.LEDMode.CURRENT_PIPELINE);
    limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
  }

  /**
   * A Pose2d representing the alignment error with this vision target. A Pose2d of (0, 0, 0) means
   * the target is perfectly aligned.
   *
   * <p>You can assume that the data in NetworkTables is for this vision target.
   */
  // TODO: Using a Pose2d for this may not be the right way to go. Refer to Limelight documentation
  // for guidance on how to implement alignment and from there decide what datastructure is best to
  // represent error. https://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html
  public abstract Pose2d alignmentError();
}
