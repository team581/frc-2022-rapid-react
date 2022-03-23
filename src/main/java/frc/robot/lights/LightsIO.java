// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LightsIO extends SubsystemIO<LightsIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public int ledCount = 0;

    @Override
    public void fromLog(LogTable table) {
      ledCount = table.getInteger("LedCount", 0);
    }

    @Override
    public void toLog(LogTable table) {
      table.put("LedCount", ledCount);
    }
  }

  public void setColor(int index, Color color);

  public void flushColor();
}
