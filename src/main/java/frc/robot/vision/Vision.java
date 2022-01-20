// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.vision.targets.DriverCamera;
import frc.robot.vision.targets.PowerPort;
import frc.robot.vision.targets.UpperHub;
import frc.robot.vision.targets.VisionTarget;

/** Used for configuring the vision system. */
public class Vision {
  public final UpperHub upperHub;
  public final PowerPort powerPort;
  public final DriverCamera noVisionTarget;

  public Vision(LimelightSubsystem limelight, PhotonVisionSubsystem photonVision) {
    upperHub = new UpperHub(limelight);
    powerPort = new PowerPort(limelight);
    noVisionTarget = new DriverCamera(limelight, photonVision);
  }

  /**
   * Selects a vision target for the vision system to use. You must call this function before trying
   * to use a vision target.
   */
  public void useVisionTarget(VisionTarget visionTarget) {
    visionTarget.onSelected();
  }
}
