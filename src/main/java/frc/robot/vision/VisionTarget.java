// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.misc.util.PolarPose2d;
import java.util.Optional;

/** A vision target for the vision system. */
public abstract class VisionTarget {
  /** The index of this vision target's pipeline. */
  public final int pipeline;

  protected final VisionSubsystemBase visionSubsystem;
  /**
   * The height from the floor to this vision target, in meters.
   *
   * @see
   *     <p>The <code>h2</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  private final double heightFromFloor;

  protected VisionTarget(
      VisionSubsystemBase visionSubsystem, double heightFromFloor, int pipeline) {
    this.visionSubsystem = visionSubsystem;
    this.heightFromFloor = heightFromFloor;
    this.pipeline = pipeline;
  }

  /**
   * Calculates the distance "as the crow flies" in meters between the camera and this vision
   * target.
   *
   * <p>This value can be used for calculating the y-axis error.
   *
   * @see
   *     <p>Returns the <code>d</code> distance from this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  private double getDistance() {
    final var h1 = visionSubsystem.getHeightFromFloor();
    final var h2 = heightFromFloor;

    final var a1 = visionSubsystem.getAngleOfElevation().getRadians();
    final var a2 = Units.degreesToRadians(visionSubsystem.getY());

    return (h2 - h1) / Math.tan(a1 + a2);
  }

  private Rotation2d getRotation() {
    return Rotation2d.fromDegrees(visionSubsystem.getX());
  }

  /**
   * The translation (in polar coordinates) from the the camera to this vision target, if visible.
   */
  public Optional<PolarPose2d> getTranslationFromCamera() {
    if (!visionSubsystem.hasTargets()) {
      return Optional.empty();
    }

    final var r = getDistance();
    final var theta = getRotation();

    return Optional.of(new PolarPose2d(r, theta));
  }
}
