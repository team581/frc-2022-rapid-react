// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lifter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LifterSubsystem extends SubsystemBase {
  private static class Constants {
    // TODO: Update port numbers
    // This part shows what motor your using. The number should match to the number with the phionex
    // tuner to show what motor connects.
    private static final int MOTOR_PORT = 0;
    private static final int UPPER_LIMIT_SWITCH = 0;
    private static final int LOWER_LIMIT_SWITCH = 0;
  }

  public enum Position {
    UP,
    MIDDLE,
    DOWN,
    INVALID
  }

  // this part tells what type of motor or switch your using
  private final TalonFX motor = new TalonFX(Constants.MOTOR_PORT);
  private final DigitalInput upperLimitSwitch = new DigitalInput(Constants.UPPER_LIMIT_SWITCH);
  private final DigitalInput lowerLimitSwitch = new DigitalInput(Constants.LOWER_LIMIT_SWITCH);

  /** Creates a new LifterSubsystem. */
  public LifterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Gets the position of the lifter */
  public Position getPosition() {
    final var isUp = upperLimitSwitch.get();
    final var isDown = lowerLimitSwitch.get();

    // figuring out where the position is
    if (isUp) {
      if (isDown) {
        // Both sensors shoudln't be active at the same time
        return Position.INVALID;
      } else {
        return Position.UP;
      }
    }

    if (isDown) {
      return Position.DOWN;
    }

    return Position.MIDDLE;
  }

  public void startLifting() {
    // setting the motor power to 0.1.  0.1 for 10% of the volts
    // TODO: Tune this value
    motor.set(TalonFXControlMode.PercentOutput, 0.1);
  }

  public void startLowering() {
    // setting the motor power to -0.1 because were lowering the lifter. 0.1 for 10% of the volts
    // TODO: Tune this value
    motor.set(TalonFXControlMode.PercentOutput, -0.1);
  }

  public void stop() {
    // setting the motor power to zero
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
