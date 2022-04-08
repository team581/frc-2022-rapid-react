// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.misc.util.sensors.SensorUnitConverter;
import org.littletonrobotics.junction.Logger;

public class ArmIOSimNeos extends ArmIONeos implements ArmIO {
  /** Height in meters of the tower the arm is attached to. */
  private static final double TOWER_HEIGHT = 330.20 / 1e3;

  /** The arm tower's angle relative to the floor. */
  private static final Rotation2d TOWER_ANGLE = Rotation2d.fromDegrees(90);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          getMotorSim(),
          Arm.GEARING,
          Arm.ARM_LENGTH,
          Arm.MOMENT_OF_INERTIA,
          // This assumes that the DOWN position has an angle less than the UP position
          ArmPosition.DOWN.stateForSimulation.position,
          ArmPosition.UP.stateForSimulation.position,
          Arm.ARM_MASS,
          true);
  private final Mechanism2d arm2d =
      new Mechanism2d(Arm.ARM_LENGTH * 1.5, TOWER_HEIGHT + Arm.ARM_LENGTH);
  private final MechanismRoot2d armPivot =
      arm2d.getRoot("ArmPivot", (Arm.ARM_LENGTH * 1.5) / 4, (TOWER_HEIGHT + Arm.ARM_LENGTH) / 4);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", TOWER_HEIGHT, TOWER_ANGLE.getDegrees()));
  private final MechanismLigament2d arm =
      armTower.append(new MechanismLigament2d("Arm", Arm.ARM_LENGTH, 0));

  public ArmIOSimNeos() {
    REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));

    SmartDashboard.putData("Arm Sim", arm2d);

    armTower.setColor(new Color8Bit(Color.kBlue));
    arm.setColor(new Color8Bit(Color.kYellow));

    sim.setState(
        VecBuilder.fill(
            Arm.STARTING_POSITION.stateForSimulation.position,
            Arm.STARTING_POSITION.stateForSimulation.velocity));

    // TODO: Use CAD for drawing accurate line widths
  }

  @Override
  public void updateInputs(Inputs inputs) {
    sim.setInputVoltage(motor.getAppliedOutput());

    sim.update(Constants.PERIOD_SECONDS);

    final var positionRadians = sim.getAngleRads();
    final var velocityRadiansPerSecond = sim.getVelocityRadPerSec();

    final var encoderSim = encoder.getSimCollection();

    encoderSim.setRawPosition(
        (int)
            Math.round(
                SensorUnitConverter.cancoder.radiansToSensorUnits(
                    INITIAL_ENCODER_POSITION.getRadians() + positionRadians)));

    final var velocitySensorUnits =
        SensorUnitConverter.cancoder.radiansPerSecondToSensorUnitsPer100ms(
            velocityRadiansPerSecond);
    encoderSim.setVelocity(velocitySensorUnits);

    Logger.getInstance().recordOutput("Arm/Sim/VelocityRadiansPerSecond", velocityRadiansPerSecond);
    Logger.getInstance().recordOutput("Arm/Sim/PositionRadians", positionRadians);

    arm.setAngle(Units.radiansToDegrees(positionRadians) - TOWER_ANGLE.getDegrees());

    super.updateInputs(inputs);

    if (motor.getInverted()) {
      // A hack to make the desired voltage look the same as the actual voltage

      inputs.appliedVolts *= -1;
    }

    inputs.upwardLimitSwitchEnabled = positionRadians >= ArmPosition.UP.stateForSimulation.position;
    inputs.downwardLimitSwitchEnabled =
        positionRadians <= ArmPosition.DOWN.stateForSimulation.position;
  }

  @Override
  public void setVoltage(double volts) {
    if (motor.getInverted()) {
      // REV simulation software doesn't invert voltage, even when the motor is inverted
      volts *= -1;
    }

    super.setVoltage(volts);
  }
}
