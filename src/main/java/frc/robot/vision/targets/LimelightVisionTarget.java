// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.targets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimelightSubsystem;
import lib.limelight.Limelight;

/** A vision target for the Limelight. */
public abstract class LimelightVisionTarget extends VisionTarget {
  protected final LimelightSubsystem limelightSubsystem;
  /**
   * The height from the floor to this vision target, in meters.
   *
   * @see
   *     <p>The <code>h2</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  private final double heightFromFloor;

  protected LimelightVisionTarget(LimelightSubsystem limelightSubsystem, double heightFromFloor) {
    this.limelightSubsystem = limelightSubsystem;
    this.heightFromFloor = heightFromFloor;
  }

  @Override
  public void onSelected() {
    limelightSubsystem.limelight.setCamMode(Limelight.CamMode.VISION_PROCESSOR);
    limelightSubsystem.limelight.setLEDMode(Limelight.LEDMode.CURRENT_PIPELINE);
  }

  /**
   * Calculates the distance "as the crow flies" in meters between the Limelight and this vision
   * target.
   *
   * <p>This value can be used for calculating the y-axis error.
   *
   * @see
   *     <p>Returns the <code>d</code> distance from this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  private double distance() {
    final var h1 = limelightSubsystem.heightFromFloor;
    final var h2 = heightFromFloor;

    final var a1 = limelightSubsystem.angleOfElevation;
    final var a2 = Units.degreesToRadians(limelightSubsystem.limelight.getY());

    return (h2 - h1) / Math.tan(a1 + a2);
  }

  /**
   * Calculates the strafe distance in meters between the Limelight and this vision target.
   *
   * <p>This is the x component of the Manhattan distance between the Limelight and this vision
   * target.
   *
   * <p>This value can be used for calculating the x-axis error.
   */
  private double strafeDistance() {
    final var d = distance();
    final var theta = rotation();

    return d * Math.sin(theta.getRadians());
  }

  private Rotation2d rotation() {
    return new Rotation2d(Units.degreesToRadians(limelightSubsystem.limelight.getX()));
  }

  /**
   * A Pose2d representing the alignment of the Limelight relative to this vision target. The vision
   * target has a Pose2d of (0, 0, 0).
   *
   * <p>You can assume that the data in NetworkTables is for this vision target.
   */
  public Pose2d alignment() {
    final var x = strafeDistance();
    final var y = distance();
    final var theta = rotation();

    return new Pose2d(x, y, theta);
  }
}
