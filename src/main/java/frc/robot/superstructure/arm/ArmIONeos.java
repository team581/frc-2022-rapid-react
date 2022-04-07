// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;

public class ArmIONeos implements ArmIO {
  public static final boolean INVERTED;

  /**
   * The amount to subtract from the absolute position to ensure that an absolute position of 0
   * means the arm is in the {@link ArmPosition#DOWN down position}.
   */
  static final Rotation2d ENCODER_ABSOLUTE_POSITION_DIFFERENCE;

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        ENCODER_ABSOLUTE_POSITION_DIFFERENCE = Rotation2d.fromDegrees(237.920);
        // Positive voltage makes the arm go up
        // Negative voltage makes the arm go down
        INVERTED = false;
        break;
      default:
        throw new UnsupportedSubsystemException(ArmIONeos.class);
    }
  }

  protected final CANSparkMax motor;
  protected final CANCoder encoder;

  protected final SparkMaxLimitSwitch forwardLimitSwitch;
  protected final SparkMaxLimitSwitch reverseLimitSwitch;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        motor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANCoder(3);

        forwardLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    motor.setInverted(INVERTED);

    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.configSensorDirection(false);
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
  }

  protected static DCMotor getMotorSim() {
    return DCMotor.getNEO(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.position =
        Rotation2d.fromDegrees(encoder.getAbsolutePosition())
            .minus(ENCODER_ABSOLUTE_POSITION_DIFFERENCE);
    inputs.velocityRadiansPerSecond = Units.degreesToRadians(encoder.getVelocity());
    inputs.upperLimitSwitchEnabled = forwardLimitSwitch.isPressed();
    inputs.lowerLimitSwitchEnabled = reverseLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    if ((volts > 0 && forwardLimitSwitch.isPressed())
        || (volts < 0 && reverseLimitSwitch.isPressed())) {
      motor.setVoltage(0);
    }

    motor.setVoltage(volts);
  }
}
