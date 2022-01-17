// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    gyro.calibrate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
