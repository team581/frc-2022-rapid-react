// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

interface WheelIO extends SubsystemIO<WheelIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double voltageVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public double positionRadians = 0;
    public double velocityRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("VoltageVolts", voltageVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionRadians", positionRadians);
      table.put("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      voltageVolts = table.getDouble("VoltageVolts", voltageVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      positionRadians = table.getDouble("PositionRadians", positionRadians);
      velocityRadiansPerSecond =
          table.getDouble("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }
  }

  /** Sets the output voltage of the wheel's motor. */
  public void setVoltage(double outputVolts);
}
