// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.match_metadata;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface MatchMetadataIO extends SubsystemIO {
  public class Inputs implements LoggableInputs {
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