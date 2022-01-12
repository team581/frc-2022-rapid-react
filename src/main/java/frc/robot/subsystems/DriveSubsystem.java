// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private static final class Constants {

    public static final int FRONT_LEFT = 10;
    public static final int FRONT_RIGHT = 11;
    public static final int REAR_LEFT = 12;
    public static final int REAR_RIGHT = 13;
  }

  private final WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT);
  private final WPI_TalonFX frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT);
  private final WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.REAR_LEFT);
  private final WPI_TalonFX rearRight = new WPI_TalonFX(Constants.REAR_RIGHT);

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveTeleop(double x, double y, double z) {
    mecanumDrive.driveCartesian(y, x, z);
  }
}
