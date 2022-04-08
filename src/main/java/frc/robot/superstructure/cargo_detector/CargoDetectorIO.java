// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface CargoDetectorIO extends SubsystemIO<CargoDetectorIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public boolean hasLeftCargo = false;
    public boolean hasRightCargo = false;

    public void toLog(LogTable table) {
      table.put("HasLeftCargo", hasLeftCargo);
      table.put("HasRightCargo", hasRightCargo);
    }

    public void fromLog(LogTable table) {
      hasLeftCargo = table.getBoolean("HasLeftCargo", hasLeftCargo);
      hasRightCargo = table.getBoolean("HasRightCargo", hasRightCargo);
    }
  }
}
