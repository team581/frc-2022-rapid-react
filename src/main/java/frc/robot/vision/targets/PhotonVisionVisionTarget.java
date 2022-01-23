// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import frc.robot.subsystems.PhotonVisionSubsystem;

/** A vision target for PhotonVision. */
public class PhotonVisionVisionTarget extends VisionTarget {
  protected final PhotonVisionSubsystem photonVision;

  protected PhotonVisionVisionTarget(PhotonVisionSubsystem photonVision) {
    this.photonVision = photonVision;
  }

  @Override
  public void prepareForUse() {
    photonVision.camera.setDriverMode(false);
  }
}
