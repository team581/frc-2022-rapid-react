// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.util.Units;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwifferIO extends SubsystemIO<SwifferIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public double angularVelocityRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("Rpm", Units.radiansPerSecondToRotationsPerMinute(angularVelocityRadiansPerSecond));
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      angularVelocityRadiansPerSecond =
          Units.rotationsPerMinuteToRadiansPerSecond(
              table.getDouble(
                  "Rpm",
                  Units.radiansPerSecondToRotationsPerMinute(angularVelocityRadiansPerSecond)));
    }
  }

  /** Sets the output voltage of the flywheel's motor. */
  public void setVoltage(double volts);

  /** Zeroes the encoder position. */
  public void zeroEncoder();
}
