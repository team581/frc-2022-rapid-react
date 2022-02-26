// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwifferSubsystem extends SubsystemBase {

  private static class Constants {
    // TODO: Update port number
    // This part shows what motor your using. The number should match to the number with the phionex
    // tuner to show what motor connects.
    private static final int PORT = 0;
  }
  // this part tells what type of motor your using
  private final TalonFX motor = new TalonFX(Constants.PORT);

  /** Creates a new SwifferSubsystem. */
  public SwifferSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startSnarfing() {
    // setting the motor power to 0.1.  0.1 for 10% of the volts
    // TODO: Tune this value
    motor.set(TalonFXControlMode.PercentOutput, 0.1);
  }

  public void startShooting() {
    // setting the motor power to -0.1 because were reversing the shooting. 0.1 for 10% of the volts
    // TODO: Tune this value
    motor.set(TalonFXControlMode.PercentOutput, -0.1);
  }

  public void stop() {
    // setting the motor power to zero
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
