// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class LifterIOSimFalcon500 extends LifterIOFalcon500 implements LifterIO {
  private static final double ANGLE_OFFSET = Units.degreesToRadians(70);

  /** Height in meters of the tower the arm is attached to. */
  private static final double TOWER_HEIGHT = 330.20 / 1e3;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          getMotorSim(),
          Lifter.GEARING,
          Lifter.MOMENT_OF_INERTIA,
          Lifter.ARM_LENGTH,
          // This assumes that the DOWN position has an angle higher than the UP position
          LifterPosition.DOWN.state.position + ANGLE_OFFSET,
          LifterPosition.UP.state.position + ANGLE_OFFSET,
          Lifter.ARM_MASS,
          // WPILib doesn't support a partial gravity simulation, so we can't simulate the
          // overcentered arm which helps negate some of the effects of gravity. In practice, this
          // allows the arm to stay in the upright position without needing constant motor output.
          false);
  private final Mechanism2d lifter2d =
      new Mechanism2d(Lifter.ARM_LENGTH * 1.5, TOWER_HEIGHT + Lifter.ARM_LENGTH);
  private final MechanismRoot2d lifterPivot =
      lifter2d.getRoot(
          "ArmPivot", (Lifter.ARM_LENGTH * 1.5) / 4, (TOWER_HEIGHT + Lifter.ARM_LENGTH) / 4);
  private final MechanismLigament2d lifterTower =
      lifterPivot.append(new MechanismLigament2d("ArmTower", -TOWER_HEIGHT, -90));
  private final MechanismLigament2d lifter =
      lifterTower.append(new MechanismLigament2d("Arm", Lifter.ARM_LENGTH, 0));

  public LifterIOSimFalcon500() {
    SmartDashboard.putData("Lifter Sim", lifter2d);

    lifterTower.setColor(new Color8Bit(Color.kBlue));
    lifter.setColor(new Color8Bit(Color.kYellow));

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

    if (LifterIOFalcon500.INVERTED) {
      // TalonFX simulation software doesn't invert the encoder values you provide when setting the
      // simulated sensor position, so we manually do it if the motor is inverted
      sensorPositionRadians *= -1.0;
      velocityRadiansPerSecond *= -1.0;
    }

    simMotor.setIntegratedSensorRawPosition(
        (int) Math.round(radiansToSensorUnits(sensorPositionRadians)));
    simMotor.setIntegratedSensorVelocity(
        (int) Math.round(radiansPerSecondToSensorUnitsPer100ms(velocityRadiansPerSecond)));

    Logger.getInstance().recordOutput("Lifter/SimRadPerSec", sim.getVelocityRadPerSec());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    lifter.setAngle(Units.radiansToDegrees(sim.getAngleRads()));

    super.updateInputs(inputs);
  }
}
