// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwifferIO {
  public static class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public double beforeGearingAngularVelocityRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("BeforeGearingRpm", beforeGearingAngularVelocityRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      beforeGearingAngularVelocityRadiansPerSecond =
          table.getDouble("BeforeGearingRpm", beforeGearingAngularVelocityRadiansPerSecond);
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(Inputs inputs);

  /** Sets the output voltage of the flywheel's motor. */
  public void setVoltage(double volts);

  /** Zeroes the encoder position. */
  public void zeroEncoder();
}
