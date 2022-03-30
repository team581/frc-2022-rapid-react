// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Camera {
  /**
   * The camera's height to the floor, in meters
   *
   * @see
   *     <p>The <code>h1</code> distance in this diagram
   *     https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final double heightFromFloor;
  /**
   * The camera's angle of elevation (its pitch).
   *
   * @see
   *     <p>The <code>a1</code> angle in thi s
   *     diagramhttps://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   */
  public final Rotation2d angleOfElevation;
  /** The transform from the camera to the center of the robot. */
  public final Transform2d transformToCenterOfRobot;

  public Camera(
      double heightFromFloor, Rotation2d angleOfElevation, Transform2d transformToCenterOfRobot) {
    this.heightFromFloor = heightFromFloor;
    this.angleOfElevation = angleOfElevation;
    this.transformToCenterOfRobot = transformToCenterOfRobot;
  }
}
