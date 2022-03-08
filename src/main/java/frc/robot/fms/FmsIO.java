// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FmsIO {
  public static class Inputs implements LoggableInputs {
    public Alliance alliance = Alliance.Invalid;

    public void toLog(LogTable table) {
      table.put("Alliance", alliance.name());
    }

    public void fromLog(LogTable table) {
      alliance = Alliance.valueOf(table.getString("Alliance", alliance.name()));
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(Inputs inputs);
}
