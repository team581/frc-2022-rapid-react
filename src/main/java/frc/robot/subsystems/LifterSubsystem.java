// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LifterSubsystem extends SubsystemBase {

  private static final class Constants {

    public static final int PORT = 0;
  }

  private final TalonFX motor = new TalonFX(Constants.PORT);

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Up() {
    motor.set(ControlMode.PercentOutput, 0.25);
  }

  public void Down() {
    motor.set(ControlMode.PercentOutput, -0.25);
  }

  public void Stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
