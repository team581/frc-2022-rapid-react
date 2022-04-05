// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.misc.exceptions.UnsupportedSubsystemException;

public class ArmIONeos implements ArmIO {
  public static final boolean INVERTED;

  /** Used to make the arm in the visualization look like the actual robot. */
  protected static final Rotation2d SIM_ANGLE_OFFSET = Rotation2d.fromDegrees(70);

  static {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        INVERTED = false;
        break;
      default:
        throw new UnsupportedSubsystemException(ArmIONeos.class);
    }
  }

  protected final CANSparkMax leader;
  protected final CANSparkMax follower;
  protected final RelativeEncoder encoder;

  /** The limit switch that prevents the arm from moving too far downward. */
  protected final SparkMaxLimitSwitch downwardLimitSwitch;
  /** The limit switch that prevents the arm from moving too far upward. */
  protected final SparkMaxLimitSwitch upwardLimitSwitch;

  public ArmIONeos() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case SIM_BOT:
        leader = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        follower = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = follower.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

        downwardLimitSwitch = leader.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        upwardLimitSwitch = leader.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        break;
      default:
        throw new UnsupportedSubsystemException(this);
    }

    follower.follow(leader);

    leader.burnFlash();
    follower.burnFlash();
  }

  protected static DCMotor getMotorSim() {
    return DCMotor.getNEO(2);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.downwardLimitSwitchEnabled = downwardLimitSwitch.isPressed();
    inputs.upwardLimitSwitchEnabled = upwardLimitSwitch.isPressed();

    inputs.appliedVolts = new double[] {leader.getAppliedOutput(), follower.getAppliedOutput()};
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.tempCelcius =
        new double[] {leader.getMotorTemperature(), follower.getMotorTemperature()};
    inputs.position = new Rotation2d(Units.rotationsToRadians(encoder.getPosition()));
    inputs.velocityRadiansPerSecond = Units.rotationsToRadians(encoder.getVelocity());
  }

  @Override
  public void setVoltage(double volts) {
    if ((volts > 0 && downwardLimitSwitch.isPressed())
        || (volts < 0 && upwardLimitSwitch.isPressed())) {
      leader.setVoltage(0);
    }

    leader.setVoltage(volts);
  }
}
