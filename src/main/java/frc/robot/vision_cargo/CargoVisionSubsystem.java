// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.vision.VisionSubsystemBase;
import frc.robot.vision_cargo.CargoVisionTarget.Color;

public class CargoVisionSubsystem extends VisionSubsystemBase {
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

  private static final double HEIGHT_FROM_FLOOR;
  private static final Rotation2d ANGLE_OF_ELEVATION;

  static {
    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        HEIGHT_FROM_FLOOR = Units.inchesToMeters(15.5);
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
      default:
        HEIGHT_FROM_FLOOR = 0.25;
        ANGLE_OF_ELEVATION = new Rotation2d(0);
        break;
    }
  }

  public final LoadingBayVisionTarget loadingBay = new LoadingBayVisionTarget(this);
  private final CargoVisionTarget redCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.RED);
  private final CargoVisionTarget blueCargo =
      new CargoVisionTarget(this, CargoVisionTarget.Color.BLUE);
  private CargoVisionTarget ourCargoVisionTarget;
  private CargoVisionTarget opponentCargoVisionTarget;

  /** Creates a new CargoVisionSubsystem. */
  public CargoVisionSubsystem(CargoVisionIO io) {
    super("CargoVision", io, Pipelines.DRIVER_MODE.index);
  }

  public CargoVisionTarget getOurCargoVisionTarget() {
    return ourCargoVisionTarget;
  }

  public CargoVisionTarget getOpponentCargoVisionTarget() {
    return opponentCargoVisionTarget;
  }

  /** Sets the {@link CargoVisionTarget}s in this class based on what our team's alliance is. */
  public void setAlliances(Alliance ourAlliance) {
    if (ourAlliance == Alliance.Invalid) {
      return;
    }

    final var ourAllianceColor = ourAlliance == Alliance.Blue ? Color.BLUE : Color.RED;
    final var opponentAllianceColor = ourAlliance == Alliance.Blue ? Color.RED : Color.BLUE;

    ourCargoVisionTarget = getTargetForAlliance(ourAllianceColor);
    opponentCargoVisionTarget = getTargetForAlliance(opponentAllianceColor);
  }

  /** Gets the {@link CargoVisionTarget} instance for the provided alliance (red or blue). */
  private CargoVisionTarget getTargetForAlliance(CargoVisionTarget.Color alliance) {
    if (alliance == CargoVisionTarget.Color.RED) {
      return redCargo;
    }

    return blueCargo;
  }

  @Override
  protected Rotation2d getAngleOfElevation() {
    return ANGLE_OF_ELEVATION;
  }

  @Override
  protected double getHeightFromFloor() {
    return HEIGHT_FROM_FLOOR;
  }
}
