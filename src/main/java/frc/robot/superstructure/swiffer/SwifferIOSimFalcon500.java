// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.swiffer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.misc.util.sensors.SensorUnitConverter;
import org.littletonrobotics.junction.Logger;

public class SwifferIOSimFalcon500 extends SwifferIOFalcon500 implements SwifferIO {
  // TODO: Calculate this from CAD
  /** The flywheel's moment of inertia in kg m^2. */
  private static final double MOMENT_OF_INERTIA = 0.00032;

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              getMotorSim(), MOMENT_OF_INERTIA, gearingConverter.gearingReduction),
          getMotorSim(),
          gearingConverter.gearingReduction);

  @Override
  public void updateInputs(Inputs inputs) {
    final var simMotor = motor.getSimCollection();

    sim.setInputVoltage(simMotor.getMotorOutputLeadVoltage());

    sim.update(Constants.PERIOD_SECONDS);

    var velocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    var velocityRPM = sim.getAngularVelocityRPM();

    if (isInverted) {
      // TalonFX simulation software doesn't invert the encoder values you provide when setting the
      // simulated sensor position/velocity, so we manually do it if the motor is inverted
      velocityRadiansPerSecond *= -1;
      velocityRPM *= -1;
    }

    final var velocitySensorUnits =
        SensorUnitConverter.talonFX.radiansPerSecondToSensorUnitsPer100ms(
            gearingConverter.afterToBeforeGearing(velocityRadiansPerSecond));

    simMotor.setIntegratedSensorVelocity(velocitySensorUnits);

    Logger.getInstance().recordOutput("Swiffer/Sim/Rpm", velocityRPM);

    super.updateInputs(inputs);
  }
}
