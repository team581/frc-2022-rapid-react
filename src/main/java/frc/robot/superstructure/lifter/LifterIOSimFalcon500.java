// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.system.plant.DCMotor;
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

public class LifterIOSimFalcon500 extends LifterIOFalcon500 implements LifterIO {
  /** Mass of arm in kilograms. */
  // TODO: Update from CAD once arm is finalized
  private static final double MASS = 1.5;

  /** Height of arm in meters. */
  // TODO: Update from CAD once arm is finalized
  private static final double HEIGHT = 0.811;

  /** Length of arm in meters. */
  // TODO: Update from CAD once arm is finalized
  private static final double WIDTH = 2.076;

  // TODO: Need to seed this with the initial angle of "UP"
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          GEARING,
          SingleJointedArmSim.estimateMOI(WIDTH, MASS),
          WIDTH,
          LifterPosition.DOWN.angle.getRadians(),
          LifterPosition.UP.angle.getRadians(),
          MASS,
          true);
  private final Mechanism2d lifter2d = new Mechanism2d(1, 1);
  private final MechanismRoot2d lifterPivot = lifter2d.getRoot("ArmPivot", 0.5, 0.5);
  private final MechanismLigament2d lifterTower =
      lifterPivot.append(new MechanismLigament2d("ArmTower", -0.25 * HEIGHT, -90));
  private final MechanismLigament2d lifter =
      lifterTower.append(
          new MechanismLigament2d("Arm", 0.25 * WIDTH, 0, 6, new Color8Bit(Color.kYellow)));

  public LifterIOSimFalcon500() {
    SmartDashboard.putData("Lifter Sim", lifter2d);
    lifterTower.setColor(new Color8Bit(Color.kBlue));
  }

  @Override
  public void updateInputs(Inputs inputs) {
    super.updateInputs(inputs);

    final var simMotor = motor.getSimCollection();

    simMotor.setBusVoltage(RobotController.getBatteryVoltage());

    sim.setInputVoltage(simMotor.getMotorOutputLeadVoltage());

    sim.update(Constants.PERIOD_SECONDS);

    final var positionRadians = sim.getAngleRads();

    simMotor.setIntegratedSensorRawPosition(radiansToSensorUnits(positionRadians));
    simMotor.setIntegratedSensorVelocity(
        radiansPerSecondToSensorUnitsPer100ms(sim.getVelocityRadPerSec()));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    lifter.setAngle(Units.radiansToDegrees(positionRadians));
  }
}
