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
import frc.robot.Constants.TargetRobot;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.util.GearingConverter;

public class ArmIONeos implements ArmIO {
  /** Initial encoder position after gearing. */
  static final Rotation2d INITIAL_ENCODER_POSITION =
      new Rotation2d(Arm.STARTING_POSITION.state.position);

  protected static final GearingConverter GEARING_CONVERTER = new GearingConverter(60.0 / 15.0);

  protected final CANSparkMax motor;
  protected final CANCoder encoder;

  protected final SparkMaxLimitSwitch downwardLimitSwitch;
  protected final SparkMaxLimitSwitch upwardLimitSwitch;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        motor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANCoder(3);

        downwardLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        upwardLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    if (Constants.getRobot() == TargetRobot.COMP_BOT) {
      motor.setInverted(true);
    }

    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    encoder.configSensorDirection(false);
    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    encoder.setPosition(0);
  }

  protected static DCMotor getMotorSim() {
    return DCMotor.getNEO(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.positionRadians =
        Rotation2d.fromDegrees(GEARING_CONVERTER.beforeToAfterGearing(encoder.getPosition()))
            .plus(INITIAL_ENCODER_POSITION)
            .getRadians();
    inputs.velocityRadiansPerSecond = Units.degreesToRadians(encoder.getVelocity());
    inputs.downwardLimitSwitchEnabled = downwardLimitSwitch.isPressed();
    inputs.upwardLimitSwitchEnabled = upwardLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    if ((volts < 0 && downwardLimitSwitch.isPressed())
        || (volts > 0 && upwardLimitSwitch.isPressed())) {
      motor.setVoltage(0);
    }

    motor.setVoltage(volts);
  }
}
