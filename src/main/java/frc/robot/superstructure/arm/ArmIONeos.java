// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;

public class ArmIONeos implements ArmIO {
  public static final boolean INVERTED;

  protected static final Rotation2d SIM_ANGLE_OFFSET = Rotation2d.fromDegrees(70);

  /**
   * The offset to apply to the absolute position to ensure that an absolute position of 0 means the
   * arm is in the {@link ArmPosition down position}.
   */
  private static final Rotation2d ENCODER_ABSOLUTE_POSITION_OFFSET;

  static {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        ENCODER_ABSOLUTE_POSITION_OFFSET = SIM_ANGLE_OFFSET;
        INVERTED = true;
        break;
      default:
        throw new UnsupportedSubsystemException(ArmIONeos.class);
    }
  }

  protected final CANSparkMax leader;
  protected final CANSparkMax follower;
  protected final CANCoder encoder;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
        leader = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
        follower = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
        encoder = new CANCoder(7);
        leader.setInverted(true);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    follower.follow(leader);
  }

  @Override
  public DCMotor getMotorSim() {
    return DCMotor.getNEO(2);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.appliedVolts = new double[] {leader.getAppliedOutput(), follower.getAppliedOutput()};
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.tempCelcius =
        new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
    inputs.position =
        Rotation2d.fromDegrees(encoder.getAbsolutePosition())
            .plus(ENCODER_ABSOLUTE_POSITION_OFFSET);
    inputs.velocityRadiansPerSecond = Units.degreesToRadians(encoder.getVelocity());
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }
}
