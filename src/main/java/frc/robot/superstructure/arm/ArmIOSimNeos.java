// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.VecBuilder;
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

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          getMotorSim(),
          Arm.GEARING,
          Arm.ARM_LENGTH,
          Arm.MOMENT_OF_INERTIA,
          // This assumes that the DOWN position has an angle higher than the UP position
          ArmPosition.DOWN.state.position,
          ArmPosition.UP.state.position,
          Arm.ARM_MASS,
          true);
  private final Mechanism2d arm2d =
      new Mechanism2d(Arm.ARM_LENGTH * 1.5, TOWER_HEIGHT + Arm.ARM_LENGTH);
  private final MechanismRoot2d armPivot =
      arm2d.getRoot("ArmPivot", (Arm.ARM_LENGTH * 1.5) / 4, (TOWER_HEIGHT + Arm.ARM_LENGTH) / 4);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", -TOWER_HEIGHT, -90));
  private final MechanismLigament2d arm =
      armTower.append(new MechanismLigament2d("Arm", Arm.ARM_LENGTH, 0));

  public ArmIOSimNeos() {
    REVPhysicsSim.getInstance().addSparkMax(leader, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(follower, DCMotor.getNEO(1));

    SmartDashboard.putData("Arm Sim", arm2d);

    armTower.setColor(new Color8Bit(Color.kBlue));
    arm.setColor(new Color8Bit(Color.kYellow));

    sim.setState(
        VecBuilder.fill(
            Arm.STARTING_POSITION.state.position, Arm.STARTING_POSITION.state.velocity));

    // TODO: Use CAD for drawing accurate line widths
  }

  @Override
  public void updateInputs(Inputs inputs) {
    sim.setInputVoltage(leader.getAppliedOutput());

    sim.update(Constants.PERIOD_SECONDS);

    var positionRadians = sim.getAngleRads();
    var velocityRadiansPerSecond = sim.getVelocityRadPerSec();

    final var encoderSim = encoder.getSimCollection();

    encoderSim.setRawPosition(
        (int) Math.round(SensorUnitConverter.cancoder.radiansToSensorUnits(positionRadians)));

    final var velocitySensorUnits =
        SensorUnitConverter.cancoder.radiansPerSecondToSensorUnitsPer100ms(
            velocityRadiansPerSecond);
    encoderSim.setVelocity(velocitySensorUnits);

    Logger.getInstance().recordOutput("Arm/Sim/VelocityRadiansPerSecond", velocityRadiansPerSecond);
    Logger.getInstance().recordOutput("Arm/Sim/PositionRadians", positionRadians);

    arm.setAngle(Units.radiansToDegrees(SIM_ANGLE_OFFSET.getRadians() + positionRadians));

    super.updateInputs(inputs);

    if (ArmIONeos.INVERTED) {
      // A hack to make the desired voltage look the same as the actual voltage

      inputs.appliedVolts = new double[] {-inputs.appliedVolts[0], -inputs.appliedVolts[1]};
    }
  }

  @Override
  public void setVoltage(double volts) {
    if (ArmIONeos.INVERTED) {
      // REV simulation software doesn't invert voltage, even when the motor is inverted
      volts *= -1;
    }

    // REV simulation software doesn't support follower motors
    follower.setVoltage(volts);
    super.setVoltage(volts);
  }
}
