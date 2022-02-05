// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.targets.Cargo;
import frc.robot.vision.targets.LoadingBay;
import frc.robot.vision.targets.UpperHub;
import lib.limelight.Limelight;

public class LimelightSubsystem extends SubsystemBase {
  public final Limelight limelight = new Limelight();

  /**
   * The angle of elevation of this Limelight, in radians.
   *
   * @see
   *     <p>The <code>a1</code> angle in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double angleOfElevation;
  /**
   * The height from the floor to this Limelight, in meters.
   *
   * @see
   *     <p>The <code>h1</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double heightFromFloor;

  // TODO: This subsystem should expose a bool when it's currently controlling the motors

  public final UpperHub upperHub = new UpperHub(this);
  public final LoadingBay loadingBay = new LoadingBay(this);
  public final Cargo redCargo = new Cargo(this, Cargo.Color.RED);
  public final Cargo blueCargo = new Cargo(this, Cargo.Color.BLUE);

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(double angleOfElevation, double heightFromFloor) {
    this.angleOfElevation = angleOfElevation;
    this.heightFromFloor = heightFromFloor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Enables raw camera output and disables computer processing. */
  public void useDriverMode() {
    limelight.setCamMode(Limelight.CamMode.DRIVER_CAMERA);
    limelight.setPipeline(9);
  }
}
