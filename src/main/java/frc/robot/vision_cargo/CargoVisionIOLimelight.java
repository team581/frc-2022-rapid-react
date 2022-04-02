// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision_cargo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.vision.VisionIOLimelight;
import lib.limelight.Limelight;

public class CargoVisionIOLimelight extends VisionIOLimelight implements CargoVisionIO {
  public CargoVisionIOLimelight() {
    super(new Limelight("limelight-cargo"));

    switch (Constants.getRobot()) {
      case TEST_2020_BOT:
        heightFromFloor = Units.inchesToMeters(15.5);
        angleOfElevation = Rotation2d.fromDegrees(0.0);
        break;
      default:
        heightFromFloor = 0.5;
        angleOfElevation = new Rotation2d();
        break;
    }
  }
}
