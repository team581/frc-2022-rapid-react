// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

interface ArmIO extends SubsystemIO<ArmIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0;
    public boolean upperLimitSwitchEnabled = false;
    public boolean lowerLimitSwitchEnabled = false;

    public void toLog(LogTable table) {
      table.put("VoltageVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionRadians", position.getRadians());
      table.put("VelocityRadiansPerSecond", velocityRadiansPerSecond);
      table.put("UpperLimitSwitchEnabled", upperLimitSwitchEnabled);
      table.put("LowerLimitSwitchEnabled", lowerLimitSwitchEnabled);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("VoltageVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      position = new Rotation2d(table.getDouble("PositionRadians", position.getRadians()));
      velocityRadiansPerSecond =
          table.getDouble("VelocityRadiansPerSecond", velocityRadiansPerSecond);
      upperLimitSwitchEnabled =
          table.getBoolean("UpperLimitSwitchEnabled", upperLimitSwitchEnabled);
      lowerLimitSwitchEnabled =
          table.getBoolean("LowerLimitSwitchEnabled", lowerLimitSwitchEnabled);
    }
  }

  /** Sets the output voltage of the arm's motor. */
  public void setVoltage(double volts);
}
