// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.targets.Cargo;
import frc.robot.vision.targets.LoadingBay;

public class CargoLimelightSubsystem extends LimelightSubsystemBase {
  public final LoadingBay loadingBay = new LoadingBay(this);
  public final Cargo redCargo = new Cargo(this, Cargo.Color.RED);
  public final Cargo blueCargo = new Cargo(this, Cargo.Color.BLUE);

  /** Creates a new CargoLimelightSubsystem. */
  public CargoLimelightSubsystem() {
    super("cargo", 0.0, Units.inchesToMeters(15.5));
  }
}
