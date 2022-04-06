// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;
import frc.robot.misc.util.GearingConverter;
import frc.robot.misc.util.sensors.SensorUnitConverter;
import org.littletonrobotics.junction.Logger;

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
        ENCODER_ABSOLUTE_POSITION_DIFFERENCE = Rotation2d.fromDegrees(0);
        INVERTED = false;
        break;
      default:
        throw new UnsupportedSubsystemException(ArmIONeos.class);
    }
  }

  protected final CANSparkMax motor;
  protected final RelativeEncoder encoder;

  protected final SparkMaxLimitSwitch forwardLimitSwitch;
  protected final SparkMaxLimitSwitch reverseLimitSwitch;
  private final GearingConverter gearingConverter;

  private double previousPositionRadians = Arm.STARTING_POSITION.state.position;
  private double previousTimestamp = -1;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        motor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        forwardLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        gearingConverter = new GearingConverter(300.0 / 7.0);

        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    motor.setInverted(INVERTED);
  }

  protected static DCMotor getMotorSim() {
    return DCMotor.getNEO(1);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    final var currentPositionRadians =
        gearingConverter.beforeToAfterGearing(
            SensorUnitConverter.sparkMAX.sensorUnitsToRadians(encoder.getPosition()));

    final var currentTimestamp = Logger.getInstance().getTimestamp();
    double velocityRadiansPerSecond;

    if (previousTimestamp == -1) {
      // We assume a velocity of 0 on the first run
      velocityRadiansPerSecond = 0;
    } else {

      final var positionDelta = currentPositionRadians - previousPositionRadians;
      final var timestampDelta = currentTimestamp - previousTimestamp;

      velocityRadiansPerSecond = positionDelta / timestampDelta;
    }

    previousPositionRadians = currentPositionRadians;
    previousTimestamp = currentTimestamp;

    inputs.appliedVolts = motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.tempCelcius = motor.getMotorTemperature();
    inputs.position = new Rotation2d(currentPositionRadians);
    inputs.velocityRadiansPerSecond = velocityRadiansPerSecond;
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
