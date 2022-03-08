// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

/** The corner of a wheel. */
public enum Corner {
  FRONT_RIGHT("FrontRight"),
  FRONT_LEFT("FrontLeft"),
  REAR_RIGHT("RearRight"),
  REAR_LEFT("RearLeft");

  private final String displayName;

  private Corner(String displayName) {
    this.displayName = displayName;
  }

  @Override
  public String toString() {
    return displayName;
  }
}
