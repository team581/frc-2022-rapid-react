// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util.sensors;

public class CancoderConverter extends SensorUnitConverterBase {
  public CancoderConverter() {
    super(4096);
  }

  public int radiansPerSecondToSensorUnitsPer100ms(double radiansPerSecond) {
    return (int) Math.round(radiansToSensorUnits(radiansPerSecond) / 10);
  }

  public double sensorUnitsPer100msToRadiansPerSecond(double sensorUnitsPer100ms) {
    return sensorUnitsToRadians(sensorUnitsPer100ms * 10);
  }
}
