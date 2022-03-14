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
import frc.robot.match_metadata.*;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.paths.commands.SimplePathCommand;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.LifterDownAndSnarfCommand;
import frc.robot.superstructure.commands.LifterUpAndSwifferShootCommand;
import frc.robot.superstructure.commands.LifterUpAndSwifferStopCommand;
import frc.robot.superstructure.lifter.*;
import frc.robot.superstructure.swiffer.*;
import frc.robot.vision.commands.LoadingBayAlignCommand;
import frc.robot.vision_cargo.*;
import frc.robot.vision_upper.*;

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

  private final MatchMetadataSubsystem matchMetadataSubsystem;
  private final ImuSubsystem imuSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final UpperHubVisionSubsystem upperVisionSubsystem;
  private final CargoVisionSubsystem cargoVisionSubsystem;
  private final Swiffer swiffer;
  private final Lifter lifter;
  private final SuperstructureSubsystem superstructureSubsystem;

  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOReplay());
      lifter = new Lifter(new LifterIOReplay());
      swiffer = new Swiffer(new SwifferIOReplay());
      imuSubsystem = new ImuSubsystem(new ImuIOReplay());
      driveSubsystem =
          new DriveSubsystem(
              driverController,
              imuSubsystem::getRotation,
              new WheelIOReplay(Corner.FRONT_LEFT),
              new WheelIOReplay(Corner.FRONT_RIGHT),
              new WheelIOReplay(Corner.REAR_LEFT),
              new WheelIOReplay(Corner.REAR_RIGHT));
      upperVisionSubsystem = new UpperHubVisionSubsystem(new UpperHubVisionIOReplay());
      cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOReplay());
    } else {
      switch (Constants.getRobot()) {
        case COMP_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOFms());
          lifter = new Lifter(new LifterIOReplay());
          swiffer = new Swiffer(new SwifferIOReplay());
          imuSubsystem = new ImuSubsystem(new ImuIONavx());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOFalcon500(Corner.FRONT_LEFT),
                  new WheelIOFalcon500(Corner.FRONT_RIGHT),
                  new WheelIOFalcon500(Corner.REAR_LEFT),
                  new WheelIOFalcon500(Corner.REAR_RIGHT));
          upperVisionSubsystem = new UpperHubVisionSubsystem(new UpperHubVisionIOReplay());
          cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOReplay());
          break;
        case TEST_2020_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOFms());
          lifter = new Lifter(new LifterIOReplay());
          swiffer = new Swiffer(new SwifferIOReplay());
          imuSubsystem = new ImuSubsystem(new ImuIOAdis16470());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOFalcon500(Corner.FRONT_LEFT),
                  new WheelIOFalcon500(Corner.FRONT_RIGHT),
                  new WheelIOFalcon500(Corner.REAR_LEFT),
                  new WheelIOFalcon500(Corner.REAR_RIGHT));
          upperVisionSubsystem = new UpperHubVisionSubsystem(new UpperHubVisionIOReplay());
          cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOLimelight());
          break;
        case SIM_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOSim());
          lifter = new Lifter(new LifterIOSimFalcon500());
          swiffer = new Swiffer(new SwifferIOSim());
          imuSubsystem = new ImuSubsystem(new ImuIOSim());
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem::getRotation,
                  new WheelIOSim(Corner.FRONT_LEFT),
                  new WheelIOSim(Corner.FRONT_RIGHT),
                  new WheelIOSim(Corner.REAR_LEFT),
                  new WheelIOSim(Corner.REAR_RIGHT));
          upperVisionSubsystem = new UpperHubVisionSubsystem(new UpperHubVisionIOSim());
          cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOSim());
          break;
        default:
          throw new UnknownTargetRobotException();
      }
    }

    superstructureSubsystem = new SuperstructureSubsystem(swiffer, lifter);

    // Configure the button bindings. You must call this after the subsystems are defined since they
    // are used to add command requirements.
    configureDriverButtonBindings();
    configureCopilotButtonBindings();

    autoCommand =
        new ParallelCommandGroup(new LoadingBayAlignCommand(driveSubsystem, cargoVisionSubsystem));
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
        new LoadingBayAlignCommand(driveSubsystem, cargoVisionSubsystem));

    // Snarfing
    copilotController
        .rightTrigger
        .whenPressed(new LifterDownAndSnarfCommand(superstructureSubsystem))
        .whenReleased(new LifterUpAndSwifferStopCommand(superstructureSubsystem));

    // Shooting
    copilotController
        .leftTrigger
        .whenPressed(new LifterUpAndSwifferShootCommand(superstructureSubsystem))
        .whenReleased(new LifterUpAndSwifferStopCommand(superstructureSubsystem));
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
