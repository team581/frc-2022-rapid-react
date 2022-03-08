// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.wheel;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface WheelIO extends SubsystemIO {
  public class Inputs implements LoggableInputs {
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
    public double positionRadians = 0;
    public double velocityRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionRadians", positionRadians);
      table.put("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDouble("CurrentAmps", currentAmps);
      tempCelcius = table.getDouble("TempCelcius", tempCelcius);
      positionRadians = table.getDouble("PositionRadians", positionRadians);
      velocityRadiansPerSecond =
          table.getDouble("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(Inputs inputs);

  /** Sets the output voltage of the wheel's motor. */
  public void setVoltage(double outputVolts);

  /** Zeroes the encoder position. */
  public void zeroEncoder();

  public MotorController getMotorController();
}
