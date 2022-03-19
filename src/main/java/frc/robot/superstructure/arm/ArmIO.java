// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.misc.SubsystemIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO extends SubsystemIO<ArmIO.Inputs> {
  public class Inputs implements LoggableInputs {
    public double[] appliedVolts = {};
    public double[] currentAmps = {};
    public double[] tempCelcius = {};
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0;

    public void toLog(LogTable table) {
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("PositionRadians", position.getRadians());
      table.put("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }

    public void fromLog(LogTable table) {
      appliedVolts = table.getDoubleArray("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      position = new Rotation2d(table.getDouble("PositionRadians", position.getRadians()));
      velocityRadiansPerSecond =
          table.getDouble("VelocityRadiansPerSecond", velocityRadiansPerSecond);
    }
  }

  public DCMotor getMotorSim();

  /** Sets the output voltage of the arm's motor. */
  public void setVoltage(double volts);
}
