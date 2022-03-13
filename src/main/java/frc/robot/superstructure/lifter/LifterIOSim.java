// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.lifter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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

// Talon FX already has a good built-in simulation system, so we can just extend the Falcon 500 IO
// class
public class LifterIOSim implements LifterIO {
  /** Mass of arm in kilograms. */
  // TODO: Update from CAD once arm is finalized
  private static final double MASS = 6.5;

  /** Height of arm in meters. */
  // TODO: Update from CAD once arm is finalized
  private static final double HEIGHT = 0.811;

  /** Length of arm in meters. */
  // TODO: Update from CAD once arm is finalized
  private static final double WIDTH = 2.076;

  /** Gearing of the arm. */
  private static final double GEARING_RATIO = 5;

  /** A scalar value to convert meters to pixels for rendering in the simulation UI. */
  private static final double SIM_SCALAR = 30;

  private final DCMotor motor = DCMotor.getFalcon500(1);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motor,
          GEARING_RATIO,
          SingleJointedArmSim.estimateMOI(WIDTH, MASS),
          WIDTH,
          Double.NEGATIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          MASS,
          true);

  private final Mechanism2d lifter2d = new Mechanism2d(WIDTH * SIM_SCALAR, HEIGHT * SIM_SCALAR * 4);
  private final MechanismRoot2d lifterPivot =
      lifter2d.getRoot("ArmPivot", (WIDTH * SIM_SCALAR) / 2, 0);
  private final MechanismLigament2d lifterTower =
      lifterPivot.append(new MechanismLigament2d("ArmTower", HEIGHT * SIM_SCALAR, 90));
  private final MechanismLigament2d lifter =
      lifterTower.append(new MechanismLigament2d("Arm", WIDTH * SIM_SCALAR, 0));

  private double desiredVolts = 0;
  /** The angle offset in radians. */
  private double angleOffset = 0;

  public LifterIOSim() {
    SmartDashboard.putData("Lifter Sim", lifter2d);
    lifterTower.setColor(new Color8Bit(Color.kBlue));
  }

  @Override
  public void setVoltage(double volts) {
    desiredVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void zeroEncoder() {
    angleOffset = sim.getAngleRads();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    sim.setInput(desiredVolts);

    sim.update(Constants.PERIOD_SECONDS);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    lifter.setAngle(Units.radiansToDegrees((sim.getAngleRads() - angleOffset)));

    inputs.appliedVolts = desiredVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelcius = 0;
    inputs.positionRadians = sim.getAngleRads() - angleOffset;
    inputs.velocityRadiansPerSecond = sim.getVelocityRadPerSec();
  }
}
