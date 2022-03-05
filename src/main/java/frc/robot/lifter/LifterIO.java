// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LifterIO {
  public static class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public boolean upperLimitSwitchActive = false;
    public boolean lowerLimitSwitchActive = false;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("UpperLimitSwitchActive", upperLimitSwitchActive);
      table.put("LowerLimitSwitchActive", lowerLimitSwitchActive);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      upperLimitSwitchActive = table.getBoolean("UpperLimitSwitchActive", upperLimitSwitchActive);
      lowerLimitSwitchActive = table.getBoolean("LowerLimitSwitchActive", lowerLimitSwitchActive);
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(Inputs inputs);

  /** Sets the lifter's motor voltage as a percentage. */
  public void setMotorPercentage(double percentage);
}
