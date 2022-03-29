// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.DriveController;
import frc.robot.drive.commands.TeleopDriveCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase {
  private static final class Constants {
    // Max of 1 rotation per second and max acceleration of 0.5 rotations
    // per second squared
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180));

    /** The acceptable amount of error between the robot's current pose and the desired pose. */
    private static final Pose2d POSE_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5));
  }

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          1, 0, 0, Constants.MAX_ROTATION, frc.robot.Constants.PERIOD_SECONDS);

  private final Drivebase drivebase;
  private final Supplier<Rotation2d> rotationSupplier;

  // Used for following trajectories
  public final HolonomicDriveController driveController =
      new HolonomicDriveController(
          // X controller
          new PIDController(1, 0, 0, frc.robot.Constants.PERIOD_SECONDS),
          // Y controller
          new PIDController(1, 0, 0, frc.robot.Constants.PERIOD_SECONDS),
          thetaController);

  public final MecanumDriveKinematics kinematics;

  private final MecanumDriveOdometry odometry;

  public final TrajectoryConfig trajectoryConfig;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(
      DriveController controller,
      Supplier<Rotation2d> rotationSupplier,
      WheelIO frontLeftIO,
      WheelIO frontRightIO,
      WheelIO rearLeftIO,
      WheelIO rearRightIO) {
    this.rotationSupplier = rotationSupplier;
    drivebase = new Drivebase(frontLeftIO, frontRightIO, rearLeftIO, rearRightIO);

    kinematics =
        new MecanumDriveKinematics(
            drivebase.frontLeft.positionToCenterOfRobot,
            drivebase.frontRight.positionToCenterOfRobot,
            drivebase.rearLeft.positionToCenterOfRobot,
            drivebase.rearRight.positionToCenterOfRobot);
    odometry = new MecanumDriveOdometry(kinematics, rotationSupplier.get());
    trajectoryConfig =
        new TrajectoryConfig(Drivebase.MAX_VELOCITY, Drivebase.MAX_ACCELERATION)
            .setKinematics(kinematics);

    setDefaultCommand(new TeleopDriveCommand(this, controller));

    driveController.setTolerance(Constants.POSE_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    drivebase.periodic();

    odometry.update(rotationSupplier.get(), drivebase.getWheelSpeeds());

    final var pose = getPose();
    // The robot's position
    Logger.getInstance()
        .recordOutput(
            "Drive/RobotPose",
            new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drivebase.setCartesianPercentages(
        xPercentage, yPercentage, thetaPercentage, rotationSupplier.get());
  }

  /** Stops all the motors. */
  public void stopMotors() {
    drivebase.stopMotors();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(drivebase.getWheelSpeeds());
  }

  public double getLinearVelocity() {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    return Math.sqrt(
        Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    final var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    setWheelSpeeds(wheelSpeeds);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return drivebase.getWheelSpeeds();
  }

  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    drivebase.setWheelSpeeds(wheelSpeeds);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Used for showing a ghost robot of the expected position while following a trajectory. For
   * comparing with the actual position.
   */
  public void logTrajectoryPose(Trajectory.State state) {
    Logger.getInstance()
        .recordOutput(
            "Drive/ExpectedRobotPose",
            new double[] {
              state.poseMeters.getX(),
              state.poseMeters.getY(),
              state.poseMeters.getRotation().getRadians()
            });
  }

  /** Resets sensors to prepare for following a trajectory using its initial state. */
  public void resetSensorsForTrajectory(Trajectory.State initialTrajectoryState) {
    drivebase.zeroEncoders();
    // TODO: This maybe should incorporate the inital state's start rotation
    odometry.resetPosition(initialTrajectoryState.poseMeters, rotationSupplier.get());
  }

  /** Resets sensors to prepare for following a trajectory. */
  public void resetSensorsForTrajectory(PathPlannerTrajectory trajectory) {
    resetSensorsForTrajectory(trajectory.getInitialState());
  }
}
