// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controller.ControllerUtil;
import frc.robot.drive.commands.TeleopDriveCommand;
import frc.robot.imu.ImuSubsystem;
import io.github.oblarg.oblog.Loggable;

/**
 * A high-level interface for the drivetrain.
 *
 * <p>Allows you to control all the wheels as a group (via {@link Drivebase}) as well as kinematics,
 * odometry, and trajectory helpers.
 */
public class DriveSubsystem extends SubsystemBase implements Loggable {
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(1, 0, 0, Constants.MAX_ROTATION);

  private static final class Constants {
    // Max of 1 rotation per second and max acceleration of 0.5 rotations
    // per second squared
    private static final TrapezoidProfile.Constraints MAX_ROTATION =
        new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180));

    /** The acceptable amount of error between the robot's current pose and the desired pose. */
    private static final Pose2d POSE_TOLERANCE = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(5));
  }

  private final Field2d field = new Field2d();

  // Components of the drive subsystem to reduce how huge this file is
  private final Drivebase drivebase = new Drivebase();
  private final ImuSubsystem imu;

  // Used for following trajectories
  public final HolonomicDriveController driveController =
      new HolonomicDriveController(
          // X controller
          new PIDController(1, 0, 0),
          // Y controller
          new PIDController(1, 0, 0),
          thetaController);

  public final MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          drivebase.frontLeft.positionToCenterOfRobot,
          drivebase.frontRight.positionToCenterOfRobot,
          drivebase.rearLeft.positionToCenterOfRobot,
          drivebase.rearRight.positionToCenterOfRobot);

  private final MecanumDriveOdometry odometry;

  public final TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(Drivebase.Constants.MAX_VELOCITY, Drivebase.Constants.MAX_ACCELERATION)
          .setKinematics(kinematics);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ControllerUtil controller, ImuSubsystem imuSubsystem) {
    imu = imuSubsystem;

    odometry = new MecanumDriveOdometry(kinematics, imu.getRotation());

    setDefaultCommand(new TeleopDriveCommand(this, controller));

    driveController.setTolerance(Constants.POSE_TOLERANCE);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();
  }

  public void driveTeleop(double xPercentage, double yPercentage, double thetaPercentage) {
    drivebase.setCartesianPercentages(
        xPercentage, yPercentage, thetaPercentage, imu.getRotation());
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

  public void setCurrentTrajectory(Trajectory trajectory) {
    // TODO: Seems like this doesn't work
    field.getObject("traj").setTrajectory(trajectory);
  }

  /** Updates odometry using sensor data. */
  private void updateOdometry() {
    odometry.update(imu.getRotation(), drivebase.getWheelSpeeds());
    field.setRobotPose(odometry.getPoseMeters());
  }

  /** Resets sensors to prepare for following a trajectory using its initial state. */
  public void resetSensorsForTrajectory(PathPlannerState initialTrajectoryState) {
    drivebase.zeroEncoders();
    // TODO: This maybe should incorporate the inital state's start rotation
    odometry.resetPosition(initialTrajectoryState.poseMeters, imu.getRotation());
  }

  /** Resets sensors to prepare for following a trajectory. */
  public void resetSensorsForTrajectory(PathPlannerTrajectory trajectory) {
    resetSensorsForTrajectory(trajectory.getInitialState());
  }
}
