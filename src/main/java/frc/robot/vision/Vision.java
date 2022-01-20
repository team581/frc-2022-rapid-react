// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.vision.targets.PowerPort;
import frc.robot.vision.targets.UpperHub;
import frc.robot.vision.targets.VisionTarget;

/** Used for configuring the vision system. */
public class Vision {
  public final UpperHub upperHub;
  public final PowerPort powerPort;

  public Vision(LimelightSubsystem limelight, PhotonVisionSubsystem photonVision) {
    upperHub = new UpperHub(limelight);
    powerPort = new PowerPort(limelight);
  }

  public enum CornerCoords {
    /** Top left corner of the vision target */
    TOP_LEFT_X(0),
    /** Top left corner of the vision target */
    TOP_LEFT_Y(1),
    /** Top right corner of the vision target */
    TOP_RIGHT_X(2),
    /** Top right corner of the vision target */
    TOP_RIGHT_Y(3),
    /** Bottom right corner of the vision target */
    BOTTOM_RIGHT_X(4),
    /** Bottom right corner of the vision target */
    BOTTOM_RIGHT_Y(5),
    /** Bottom left corner of the vision target */
    BOTTOM_LEFT_X(6),
    /** Bottom left corner of the vision target */
    BOTTOM_LEFT_Y(7);

    public final int value;

    CornerCoords(final int value) {
      this.value = value;
    }
  }

  /**
   * Selects a vision target for the vision system to use. You must call this function before trying
   * to use a vision target.
   */
  public void useVisionTarget(VisionTarget visionTarget) {
    visionTarget.onSelected();
  }
}
