// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util.sensors;

public class SensorUnitConverter {
  public static final CancoderConverter cancoder = new CancoderConverter();
  public static final TalonFXConverter talonFX = new TalonFXConverter();

  private SensorUnitConverter() {}
}
