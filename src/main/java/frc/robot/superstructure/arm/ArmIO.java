// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

interface ArmIO extends SubsystemIO<ArmIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public double positionRadians = 0;
    public double velocityRadiansPerSecond = 0;
    public boolean downwardLimitSwitchEnabled = false;
    public boolean upwardLimitSwitchEnabled = false;

    public void toLog(LogTable table) {
      table.put("VoltageVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionRadians", positionRadians);
      table.put("VelocityRadiansPerSecond", velocityRadiansPerSecond);
      table.put("DownwardLimitSwitchEnabled", downwardLimitSwitchEnabled);
      table.put("UpwardLimitSwitchEnabled", upwardLimitSwitchEnabled);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("VoltageVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      positionRadians = table.getDouble("PositionRadians", positionRadians);
      velocityRadiansPerSecond =
          table.getDouble("VelocityRadiansPerSecond", velocityRadiansPerSecond);
      downwardLimitSwitchEnabled =
          table.getBoolean("DownwardLimitSwitchEnabled", downwardLimitSwitchEnabled);
      upwardLimitSwitchEnabled =
          table.getBoolean("UpwardLimitSwitchEnabled", upwardLimitSwitchEnabled);
    }
  }

  /** Sets the output voltage of the arm's motor. */
  public void setVoltage(double volts);
}
