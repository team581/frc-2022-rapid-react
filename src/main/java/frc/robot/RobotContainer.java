// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controller.ControllerUtil;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.commands.VelocityControlTestCommand;
import frc.robot.gyro.*;
import frc.robot.lifter.*;
import frc.robot.limelight_cargo.CargoLimelightSubsystem;
import frc.robot.limelight_upper.UpperHubLimelightSubsystem;
import frc.robot.misc.commands.RefreshAllianceWithFmsCommand;
import frc.robot.paths.commands.SimplePathCommand;
import frc.robot.swiffer.*;
import frc.robot.swiffer.commands.StartShootingCommand;
import frc.robot.swiffer.commands.StartSnarfingCommand;
import frc.robot.swiffer.commands.StopSwifferCommand;
import frc.robot.vision.commands.LoadingBayAlignCommand;
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

  private final GyroSubsystem gyroSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final UpperHubLimelightSubsystem upperLimelightSubsystem =
      new UpperHubLimelightSubsystem();
  private final CargoLimelightSubsystem cargoLimelightSubsystem = new CargoLimelightSubsystem();
  private final SwifferSubsystem swifferSubsystem;
  private final LifterSubsystem lifterSubsystem;

  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    if (Constants.getMode() == Constants.Mode.REPLAY) {
      lifterSubsystem = new LifterSubsystem(new LifterIOReplay());
      swifferSubsystem = new SwifferSubsystem(new SwifferIOReplay());
      gyroSubsystem = new GyroSubsystem(new GyroIOReplay());
    } else {
      switch (Constants.getRobot()) {
        case COMP_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOReal());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOReal());
          gyroSubsystem = new GyroSubsystem(new GyroIONavx());
          break;
        case TEST_2020_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOReplay());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOReplay());
          gyroSubsystem = new GyroSubsystem(new GyroIONavx());
          break;
        case SIM_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOSim());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOSim());
          gyroSubsystem = new GyroSubsystem(new GyroIOSim());
          break;
        default:
          throw new IllegalStateException("Unknown target robot");
      }
    }

    driveSubsystem = new DriveSubsystem(controllerUtil, gyroSubsystem);

    autoCommand =
        new SequentialCommandGroup(
            new RefreshAllianceWithFmsCommand(cargoLimelightSubsystem),
            new LoadingBayAlignCommand(driveSubsystem, cargoLimelightSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO: Instantiate all buttons in a `Controller` class which will be subclassed by
    // `DriverController` and `CopilotController`

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
        .whenPressed(new StartSnarfingCommand(swifferSubsystem, lifterSubsystem))
        .whenReleased(new StopSwifferCommand(swifferSubsystem));
    leftTrigger
        .whenPressed(new StartShootingCommand(swifferSubsystem, lifterSubsystem))
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
