// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSimFalcon500 extends ArmIOFalcon500 implements ArmIO {
  private static final double ANGLE_OFFSET = Units.degreesToRadians(70);

  /** Height in meters of the tower the arm is attached to. */
  private static final double TOWER_HEIGHT = 330.20 / 1e3;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          getMotorSim(),
          Arm.GEARING,
          Arm.MOMENT_OF_INERTIA,
          Arm.ARM_LENGTH,
          // This assumes that the DOWN position has an angle higher than the UP position
          ArmPosition.DOWN.state.position + ANGLE_OFFSET,
          ArmPosition.UP.state.position + ANGLE_OFFSET,
          Arm.ARM_MASS,
          // WPILib doesn't support a partial gravity simulation, so we can't simulate the
          // overcentered arm which helps negate some of the effects of gravity. In practice, this
          // allows the arm to stay in the upright position without needing constant motor output.
          false);
  private final Mechanism2d arm2d =
      new Mechanism2d(Arm.ARM_LENGTH * 1.5, TOWER_HEIGHT + Arm.ARM_LENGTH);
  private final MechanismRoot2d armPivot =
      arm2d.getRoot("ArmPivot", (Arm.ARM_LENGTH * 1.5) / 4, (TOWER_HEIGHT + Arm.ARM_LENGTH) / 4);
  private final MechanismLigament2d armTower =
      armPivot.append(new MechanismLigament2d("ArmTower", -TOWER_HEIGHT, -90));
  private final MechanismLigament2d arm =
      armTower.append(new MechanismLigament2d("Arm", Arm.ARM_LENGTH, 0));

  public ArmIOSimFalcon500() {
    SmartDashboard.putData("Arm Sim", arm2d);

    armTower.setColor(new Color8Bit(Color.kBlue));
    arm.setColor(new Color8Bit(Color.kYellow));

    // TODO: Use CAD for drawing accurate line widths
  }

  @Override
  public void updateInputs(Inputs inputs) {
    final var simMotor = motor.getSimCollection();

    simMotor.setBusVoltage(RobotController.getBatteryVoltage());

    sim.setInputVoltage(simMotor.getMotorOutputLeadVoltage());

    sim.update(Constants.PERIOD_SECONDS);

    var sensorPositionRadians = sim.getAngleRads();
    var velocityRadiansPerSecond = sim.getVelocityRadPerSec();

    if (ArmIOFalcon500.INVERTED) {
      // TalonFX simulation software doesn't invert the encoder values you provide when setting the
      // simulated sensor position, so we manually do it if the motor is inverted
      sensorPositionRadians *= -1.0;
      velocityRadiansPerSecond *= -1.0;
    }

    simMotor.setIntegratedSensorRawPosition(
        (int) Math.round(radiansToSensorUnits(sensorPositionRadians)));
    simMotor.setIntegratedSensorVelocity(
        (int) Math.round(radiansPerSecondToSensorUnitsPer100ms(velocityRadiansPerSecond)));

    Logger.getInstance()
        .recordOutput("Arm/Sim/VelocityRadiansPerSecond", sim.getVelocityRadPerSec());
    Logger.getInstance().recordOutput("Arm/Sim/PositionRadians", sim.getAngleRads());

    arm.setAngle(Units.radiansToDegrees(sim.getAngleRads()));

    super.updateInputs(inputs);
  }
}
