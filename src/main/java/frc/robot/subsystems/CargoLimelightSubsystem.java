// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.vision.targets.CargoVisionTarget;
import frc.robot.vision.targets.LoadingBayVisionTarget;

public class CargoLimelightSubsystem extends LimelightSubsystemBase {
  public enum Pipelines {
    RED_CARGO(0),
    BLUE_CARGO(1),
    LOADING_BAY(2),
    DRIVER_MODE(9);

    public final int index;

    Pipelines(final int index) {
      this.index = index;
    }
  }

  public final LoadingBayVisionTarget loadingBay = new LoadingBayVisionTarget(this);
  public final CargoVisionTarget redCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.RED);
  public final CargoVisionTarget blueCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.BLUE);

  /** Creates a new CargoLimelightSubsystem. */
  public CargoLimelightSubsystem() {
    super("cargo", 0.0, Units.inchesToMeters(15.5));
  }
}
