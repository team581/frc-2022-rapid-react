// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.limelight.Limelight;

public class LimelightSubsystem extends SubsystemBase {
  public final Limelight limelight = new Limelight();

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Enables raw camera output and disables computer processing. */
  public void useDriverMode() {
    limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    limelight.setPipeline(9);
  }
}
