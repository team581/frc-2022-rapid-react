// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.vision.targets.Cargo;
import frc.robot.vision.targets.LoadingBay;
import frc.robot.vision.targets.UpperHub;
import frc.robot.vision.targets.VisionTarget;

/** Used for configuring the vision system. */
public class Vision {
  public final UpperHub upperHub;
  public final LoadingBay loadingBay;
  public final Cargo redCargo;
  public final Cargo blueCargo;

  public Vision(LimelightSubsystem limelight, PhotonVisionSubsystem photonVision) {
    upperHub = new UpperHub(limelight);
    loadingBay = new LoadingBay(limelight);
    redCargo = new Cargo(photonVision, Cargo.Color.RED);
    blueCargo = new Cargo(photonVision, Cargo.Color.BLUE);
  }

  /**
   * Selects a vision target for the vision system to use. You must call this function before trying
   * to use a vision target.
   */
  public void useVisionTarget(VisionTarget visionTarget) {
    visionTarget.onSelected();
  }
}
