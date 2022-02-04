// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.groups.vision.LoadingBayAlignCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SnarferSubsystem;
import frc.robot.util.ControllerUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(gyroSubsystem);
  private final LimelightSubsystem limelightSubsystem =
      new LimelightSubsystem(
          Constants.LIMELIGHT_ANGLE_OF_ELEVATION, Constants.LIMELIGHT_HEIGHT_FROM_FLOOR);
  private final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final SnarferSubsystem snarferSubsystem = new SnarferSubsystem();

  private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);
  private final ControllerUtil controllerUtil = new ControllerUtil(controller);

  private final Command autoCommand =
      new LoadingBayAlignCommand(driveSubsystem, limelightSubsystem);

  /**
   * Whether the joysticks should be ignored during teleop. This is used to prevent the drivetrain's
   * wheel velocity controllers from clashing with the driver's joystick values.
   */
  private boolean ignoreJoysticks = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final var aButton = new JoystickButton(controller, XboxController.Button.kA.value);
    final var xButton = new JoystickButton(controller, XboxController.Button.kX.value);

    final var leftTrigger = new JoystickButton(controller, XboxController.Axis.kLeftTrigger.value);
    final var rightTrigger =
        new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);

    // Align for shooting
    LoadingBayAlignCommand loadingBayAlignCommand =
        new LoadingBayAlignCommand(driveSubsystem, limelightSubsystem);
    aButton
        .whenPressed(() -> ignoreJoysticks = true)
        .whenPressed(loadingBayAlignCommand)
        .whenReleased(
            () -> {
              ignoreJoysticks = false;
              limelightSubsystem.useDriverMode();
              loadingBayAlignCommand.cancel();
            });

    // Snarfer
    leftTrigger.whenPressed(snarferSubsystem::start).whenInactive(snarferSubsystem::stop);
    xButton.whenPressed(snarferSubsystem::spit).whenInactive(snarferSubsystem::stop);

    // Shooter
    rightTrigger.whenPressed(shooterSubsystem::start).whenInactive(shooterSubsystem::stop);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoCommand;
  }

  public void driveWithJoystick() {
    if (ignoreJoysticks) {
      return;
    }

    final var x = controllerUtil.getXPercentage();
    final var y = controllerUtil.getYPercentage();
    final var theta = controllerUtil.getThetaPercentage();

    driveSubsystem.driveTeleop(x, y, theta);
  }

  /**
   * Disables the vision-powered autonomous system used during the autonomous period or while the
   * driver has a target alignment command running. This restores joystick control and will set the
   * camera(s) to disable vision processing (driver mode).
   */
  public void disableVisionAutonomous() {
    ignoreJoysticks = false;
    this.limelightSubsystem.useDriverMode();
  }
}
