// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.controller.ButtonController;
import frc.robot.controller.DriveController;
import frc.robot.drive.*;
import frc.robot.drive.commands.VelocityControlTestCommand;
import frc.robot.drive.wheel.*;
import frc.robot.imu.*;
import frc.robot.limelight_cargo.CargoLimelightSubsystem;
import frc.robot.limelight_upper.UpperHubLimelightSubsystem;
import frc.robot.misc.commands.LifterDownAndSnarfCommand;
import frc.robot.misc.commands.LifterUpAndSwifferShootCommand;
import frc.robot.misc.commands.LifterUpAndSwifferStopCommand;
import frc.robot.misc.commands.RefreshAllianceWithFmsCommand;
import frc.robot.paths.commands.SimplePathCommand;
import frc.robot.superstructure.lifter.*;
import frc.robot.superstructure.swiffer.*;
import frc.robot.superstructure.swiffer.commands.SwifferStopCommand;
import frc.robot.vision.commands.LoadingBayAlignCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveController driverController =
      new DriveController(new XboxController(Constants.DRIVER_CONTROLLER_PORT));
  private final ButtonController copilotController =
      new ButtonController(new XboxController(Constants.COPILOT_CONTROLLER_PORT));

  private final ImuSubsystem imuSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final UpperHubLimelightSubsystem upperLimelightSubsystem =
      new UpperHubLimelightSubsystem();
  private final CargoLimelightSubsystem cargoLimelightSubsystem = new CargoLimelightSubsystem();
  private final SwifferSubsystem swifferSubsystem;
  private final LifterSubsystem lifterSubsystem;

  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      lifterSubsystem = new LifterSubsystem(new LifterIOReplay());
      swifferSubsystem = new SwifferSubsystem(new SwifferIOReplay());
      imuSubsystem = new ImuSubsystem(new ImuIOReplay());
      driveSubsystem =
          new DriveSubsystem(
              driverController,
              imuSubsystem::getRotation,
              new WheelIOReplay(),
              new WheelIOReplay(),
              new WheelIOReplay(),
              new WheelIOReplay());
    } else {
      switch (Constants.getRobot()) {
        case COMP_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOReal());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOReal());
          imuSubsystem = new ImuSubsystem(new ImuIONavx());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOReal(Corner.FRONT_LEFT),
                  new WheelIOReal(Corner.FRONT_RIGHT),
                  new WheelIOReal(Corner.REAR_LEFT),
                  new WheelIOReal(Corner.REAR_RIGHT));
          break;
        case TEST_2020_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOReplay());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOReplay());
          imuSubsystem = new ImuSubsystem(new ImuIOAdis16470());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOReal(Corner.FRONT_LEFT),
                  new WheelIOReal(Corner.FRONT_RIGHT),
                  new WheelIOReal(Corner.REAR_LEFT),
                  new WheelIOReal(Corner.REAR_RIGHT));
          break;
        case SIM_BOT:
          lifterSubsystem = new LifterSubsystem(new LifterIOSim());
          swifferSubsystem = new SwifferSubsystem(new SwifferIOSim());
          imuSubsystem = new ImuSubsystem(new ImuIOSim());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOSim(Corner.FRONT_LEFT),
                  new WheelIOSim(Corner.FRONT_RIGHT),
                  new WheelIOSim(Corner.REAR_LEFT),
                  new WheelIOSim(Corner.REAR_RIGHT));
          break;
        default:
          throw new IllegalStateException("Unknown target robot");
      }
    }

    // Configure the button bindings. You must call this after the subsystems are defined since they
    // are used to add command requirements.
    configureDriverButtonBindings();
    configureCopilotButtonBindings();

    autoCommand =
        new ParallelCommandGroup(
            new RefreshAllianceWithFmsCommand(cargoLimelightSubsystem),
            new LoadingBayAlignCommand(driveSubsystem, cargoLimelightSubsystem));
  }

  private void configureDriverButtonBindings() {
    // Testing PathPlanner
    driverController.bButton.whenHeld(new SimplePathCommand(driveSubsystem));

    // Testing autonomous
    driverController.yButton.whenHeld(new VelocityControlTestCommand(driveSubsystem));

    // Resetting field oriented control
    driverController.xButton.whenActive(imuSubsystem::zeroHeading);
  }

  private void configureCopilotButtonBindings() {
    // Align for shooting
    copilotController.aButton.whenHeld(
        new LoadingBayAlignCommand(driveSubsystem, cargoLimelightSubsystem));

    // Snarfing
    copilotController
        .rightTrigger
        .whenPressed(new LifterDownAndSnarfCommand(swifferSubsystem, lifterSubsystem))
        .whenReleased(new LifterUpAndSwifferStopCommand(swifferSubsystem, lifterSubsystem));

    // Shooting
    copilotController
        .leftTrigger
        .whenPressed(new LifterUpAndSwifferShootCommand(swifferSubsystem, lifterSubsystem))
        .whenReleased(new SwifferStopCommand(swifferSubsystem));
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

  /** The command to run on robot init. */
  public Command getRobotInitCommand() {
    // We do this on robot init just in case the robot reboots in the middle of a match and the data
    // from autonomous init is missing
    return new RefreshAllianceWithFmsCommand(cargoLimelightSubsystem);
  }
}
