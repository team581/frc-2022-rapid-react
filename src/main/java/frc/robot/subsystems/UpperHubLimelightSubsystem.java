// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.vision.targets.UpperHub;

public class UpperHubLimelightSubsystem extends LimelightSubsystemBase {
  public final UpperHub upperHub = new UpperHub(this);

  /** Creates a new UpperHubLimelightSubsystem. */
  public UpperHubLimelightSubsystem() {
    // TODO: Add in the Limelight's angle of elevation and height
    super("upperhub", -1.0, -1.0);
  }
}
