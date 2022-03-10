// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.imu;

import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ImuIO extends SubsystemIO<ImuIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double tempCelcius = 0;
    public double rotationRadians = 0;
    public double turnRateRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("TempCelcius", tempCelcius);
      table.put("RotationsRadians", rotationRadians);
      table.put("TurnRateRadiansPerSecond", turnRateRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      rotationRadians = table.getDouble("RotationsRadians", rotationRadians);
      turnRateRadiansPerSecond =
          table.getDouble("TurnRateRadiansPerSecond", turnRateRadiansPerSecond);
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading();
}
