// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.cargo_detector.CargoDetectorIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class CargoDetector extends SubsystemBase {
  private CargoDetectorIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new CargoDetector. */
  public CargoDetector(CargoDetectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("CargoDetector", inputs);

    Logger.getInstance().recordOutput("CargoDetector/CargoCount", getCargoCount());
  }

  /** Returns the number of cargo held by the robot. */
  public int getCargoCount() {
    return (inputs.hasLeftCargo ? 1 : 0) + (inputs.hasRightCargo ? 1 : 0);
  }
}
