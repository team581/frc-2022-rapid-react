// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc.io;

/** A base class for IO classes using Falcon 500s to inherit from. */
public abstract class Falcon500IO {
  private static class Constants {
    public static final double SENSOR_RESOLUTION = 2048;
    // 2048 per rotation, so we divide by 1 rotation to get the units per radian
    public static final double SENSOR_UNITS_PER_RADIAN = SENSOR_RESOLUTION / (2 * Math.PI);

    private Constants() {}
  }

  private double gearing;

  protected Falcon500IO() {}

  /**
   * Sets the gearing of this motor to make {@link Falcon500IO#sensorUnitsToRadians(double)} return
   * a useful number. This is necessary because the encoder in Falcon 500s reads revolutions before
   * gearing, giving you a number which is higher than the actual number of times your mechanism has
   * rotated.
   */
  protected void setGearing(double gearing) {
    this.gearing = gearing;
  }

  /** Converts sensor units to radians with gearing reduction applied. */
  protected double sensorUnitsToRadians(double sensorUnits) {
    return (sensorUnits / Constants.SENSOR_UNITS_PER_RADIAN) / gearing;
  }

  /** Converts radians with gearing reduction applied to sensor units. */
  protected int radiansToSensorUnits(double radians) {
    return (int) Math.round(radians * gearing * Constants.SENSOR_UNITS_PER_RADIAN);
  }

  protected double sensorUnitsPer100msToRadiansPerSecond(double sensorUnitsPer100ms) {
    return radiansToSensorUnits(sensorUnitsPer100ms * 10);
  }

  protected int radiansPerSecondToSensorUnitsPer100ms(double radiansPerSecond) {
    return (int) Math.round(sensorUnitsToRadians(radiansPerSecond / 10));
  }
}
