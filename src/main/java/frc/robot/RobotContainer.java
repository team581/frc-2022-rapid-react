// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RefreshAllianceWithFmsCommand;
import frc.robot.commands.VelocityControlTestCommand;
import frc.robot.commands.groups.swiffer.StartShootingCommand;
import frc.robot.commands.groups.swiffer.StartSnarfingCommand;
import frc.robot.commands.groups.swiffer.StopSwifferCommand;
import frc.robot.commands.groups.trajectories.SimplePathCommand;
import frc.robot.commands.groups.vision.LoadingBayAlignCommand;
import frc.robot.subsystems.CargoLimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwifferSubsystem;
import frc.robot.subsystems.UpperHubLimelightSubsystem;
import frc.robot.util.ControllerUtil;
import io.github.oblarg.oblog.Loggable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
  // The robot's subsystems and commands are defined here...

  private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);
  private final ControllerUtil controllerUtil = new ControllerUtil(controller);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem(controllerUtil);
  private final UpperHubLimelightSubsystem upperLimelightSubsystem =
      new UpperHubLimelightSubsystem();
  private final CargoLimelightSubsystem cargoLimelightSubsystem = new CargoLimelightSubsystem();
  private final SwifferSubsystem swifferSubsystem = new SwifferSubsystem();

  private final Command autoCommand =
      new SequentialCommandGroup(
          new RefreshAllianceWithFmsCommand(cargoLimelightSubsystem),
          new LoadingBayAlignCommand(driveSubsystem, cargoLimelightSubsystem));

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
    final var bButton = new JoystickButton(controller, XboxController.Button.kB.value);
    final var xButton = new JoystickButton(controller, XboxController.Button.kX.value);
    final var yButton = new JoystickButton(controller, XboxController.Button.kY.value);

    final var leftTrigger = new JoystickButton(controller, XboxController.Axis.kLeftTrigger.value);
    final var rightTrigger =
        new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);

    // Align for shooting
    aButton.whenHeld(new LoadingBayAlignCommand(driveSubsystem, cargoLimelightSubsystem));

    // Testing PathPlanner
    bButton.whenHeld(new SimplePathCommand(driveSubsystem));

    // Testing autonomous
    yButton.whenHeld(new VelocityControlTestCommand(driveSubsystem));

    // Swiffer
    rightTrigger
        .whenPressed(new StartSnarfingCommand(swifferSubsystem))
        .whenReleased(new StopSwifferCommand(swifferSubsystem));
    leftTrigger
        .whenPressed(new StartShootingCommand(swifferSubsystem))
        .whenReleased(new StopSwifferCommand(swifferSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // TODO: Move this method to a dedicated AutonomousChooser class
  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
