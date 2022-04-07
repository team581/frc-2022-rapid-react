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

  static final Rotation2d INITIAL_ENCODER_POSITION =
      new Rotation2d(Arm.STARTING_POSITION.state.position);

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        // Doing this means negative voltage will lower the arm and positive voltage will raise the arm
        INVERTED = true;
        break;
      default:
        throw new UnsupportedSubsystemException(ArmIONeos.class);
    }
  }

  protected final CANSparkMax motor;
  protected final RelativeEncoder encoder;

  protected final SparkMaxLimitSwitch downwardLimitSwitch;
  protected final SparkMaxLimitSwitch upwardLimitSwitch;
  private final GearingConverter gearingConverter;

  private double previousPositionRadians = Arm.STARTING_POSITION.state.position;
  private double previousTimestamp = -1;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        motor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        downwardLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        upwardLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

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
                SensorUnitConverter.sparkMAX.sensorUnitsToRadians(encoder.getPosition()))
            + INITIAL_ENCODER_POSITION.getRadians();

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
    inputs.downwardLimitSwitchEnabled = downwardLimitSwitch.isPressed();
    inputs.upwardLimitSwitchEnabled = upwardLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    if (
    // Prevent the arm from lowering past the threshold
    (volts < 0 && downwardLimitSwitch.isPressed())
        ||
        // Prevent the arm from raising past the threshold
        (volts > 0 && upwardLimitSwitch.isPressed())) {
      motor.setVoltage(0);
    }

    motor.setVoltage(volts);
  }
}
