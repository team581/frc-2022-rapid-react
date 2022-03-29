// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.util.sensors;

import edu.wpi.first.math.util.Units;

abstract class SensorUnitConverterBase {
  private final double sensorUnitsPerRotation;

  protected SensorUnitConverterBase(double sensorUnitsPerRotation) {
    this.sensorUnitsPerRotation = sensorUnitsPerRotation;
  }

  public double sensorUnitsToRadians(double sensorUnits) {
    return Units.rotationsToRadians(sensorUnitsToRotations(sensorUnits));
  }

  public double radiansToSensorUnits(double radians) {
    return rotationsToSensorUnits(Units.radiansToRotations(radians));
  }

  private double sensorUnitsToRotations(double sensorUnits) {
    return sensorUnits / sensorUnitsPerRotation;
  }

  private double rotationsToSensorUnits(double rotations) {
    return rotations * sensorUnitsPerRotation;
  }
}
