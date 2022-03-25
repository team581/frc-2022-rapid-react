// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.cargo_detector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.cargo_detector.CargoDetectorIO.Inputs;
import org.littletonrobotics.junction.Logger;

public class CargoDetectorSubsystem extends SubsystemBase {
  private CargoDetectorIO io;
  private final Inputs inputs = new Inputs();

  /** Creates a new CargoDetectorSubsystem. */
  public CargoDetectorSubsystem(CargoDetectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("CargoDetector", inputs);
    // This method will be called once per scheduler run
  }
}
