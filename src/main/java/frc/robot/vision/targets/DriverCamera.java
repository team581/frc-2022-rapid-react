// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.lib.limelight.Limelight;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

/**
 * Not an actual vision target, selecting this this will enable raw camera output, turn off the
 * LEDs, etc. The camera will be used to help the driver see.
 */
public class DriverCamera extends VisionTarget {
  private final Limelight limelight;
  private final PhotonVisionSubsystem photonVision;

  public DriverCamera(LimelightSubsystem limelightSubsystem, PhotonVisionSubsystem photonVision) {
    this.limelight = limelightSubsystem.limelight;
    this.photonVision = photonVision;
  }

  @Override
  public void onSelected() {
    limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    limelight.setPipeline(9);

    photonVision.camera.setDriverMode(true);
  }
}
