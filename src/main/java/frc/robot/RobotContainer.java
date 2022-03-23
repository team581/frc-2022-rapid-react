// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.controller.ButtonController;
import frc.robot.controller.DriveController;
import frc.robot.controller.LogitechF310DirectInputController;
import frc.robot.drive.*;
import frc.robot.drive.commands.UpperHubAlignCommand;
import frc.robot.drive.commands.VelocityControlTestCommand;
import frc.robot.imu.*;
import frc.robot.lights.*;
import frc.robot.localization.Localization;
import frc.robot.match_metadata.*;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.paths.PPPaths;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.arm.*;
import frc.robot.superstructure.commands.ArmDownAndSnarfCommand;
import frc.robot.superstructure.commands.ArmUpAndSwifferShootCommand;
import frc.robot.superstructure.swiffer.*;
import frc.robot.vision_cargo.*;
import frc.robot.vision_upper.*;
import lib.pathplanner.PPCommand;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveController driverController =
      new DriveController(new LogitechF310DirectInputController(Constants.DRIVER_CONTROLLER_PORT));
  private final ButtonController copilotController =
      new ButtonController(
          new LogitechF310DirectInputController(Constants.COPILOT_CONTROLLER_PORT));

  private final MatchMetadataSubsystem matchMetadataSubsystem;
  private final ImuSubsystem imuSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final UpperHubVisionSubsystem upperVisionSubsystem;
  private final CargoVisionSubsystem cargoVisionSubsystem;
  private final Swiffer swiffer;
  private final Arm arm;
  private final SuperstructureSubsystem superstructureSubsystem;
  private final LightsSubsystem lightsSubsystem;
  private final Localization localization;

  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMP_BOT:
      case TEST_2020_BOT:
      case SIM_BOT:
        LoggedSystemStats.getInstance()
            .setPowerDistributionConfig(0, PowerDistribution.ModuleType.kCTRE);
        break;
      default:
        throw new UnknownTargetRobotException();
    }

    lightsSubsystem = new LightsSubsystem(new LightsIORoborio());
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOReplay());
      arm = new Arm(new ArmIOReplay());
      swiffer = new Swiffer(new SwifferIOReplay());
      imuSubsystem = new ImuSubsystem(new ImuIOReplay());
      upperVisionSubsystem =
          new UpperHubVisionSubsystem(new UpperHubVisionIOReplay(), imuSubsystem);
      cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOReplay(), imuSubsystem);
      driveSubsystem =
          new DriveSubsystem(
              driverController,
              imuSubsystem,
              new WheelIOReplay(Corner.FRONT_LEFT),
              new WheelIOReplay(Corner.FRONT_RIGHT),
              new WheelIOReplay(Corner.REAR_LEFT),
              new WheelIOReplay(Corner.REAR_RIGHT));
    } else {
      switch (Constants.getRobot()) {
        case COMP_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOFms());
          arm = new Arm(new ArmIONeos());
          swiffer = new Swiffer(new SwifferIOReplay());
          imuSubsystem = new ImuSubsystem(new ImuIONavx());
          upperVisionSubsystem =
              new UpperHubVisionSubsystem(new UpperHubVisionIOReplay(), imuSubsystem);
          cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOReplay(), imuSubsystem);
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem,
                  new WheelIOFalcon500(Corner.FRONT_LEFT),
                  new WheelIOFalcon500(Corner.FRONT_RIGHT),
                  new WheelIOFalcon500(Corner.REAR_LEFT),
                  new WheelIOFalcon500(Corner.REAR_RIGHT));
          break;
        case TEST_2020_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOFms());
          arm = new Arm(new ArmIOReplay());
          swiffer = new Swiffer(new SwifferIOReplay());
          imuSubsystem = new ImuSubsystem(new ImuIOAdis16470());
          upperVisionSubsystem =
              new UpperHubVisionSubsystem(new UpperHubVisionIOReplay(), imuSubsystem);
          cargoVisionSubsystem =
              new CargoVisionSubsystem(new CargoVisionIOLimelight(), imuSubsystem);
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem,
                  new WheelIOFalcon500(Corner.FRONT_LEFT),
                  new WheelIOFalcon500(Corner.FRONT_RIGHT),
                  new WheelIOFalcon500(Corner.REAR_LEFT),
                  new WheelIOFalcon500(Corner.REAR_RIGHT));
          break;
        case SIM_BOT:
          matchMetadataSubsystem = new MatchMetadataSubsystem(new MatchMetadataIOSim());
          arm = new Arm(new ArmIOSimNeos());
          swiffer = new Swiffer(new SwifferIOSimFalcon500());
          imuSubsystem = new ImuSubsystem(new ImuIOSim());
          upperVisionSubsystem =
              new UpperHubVisionSubsystem(new UpperHubVisionIOSim(), imuSubsystem);
          cargoVisionSubsystem = new CargoVisionSubsystem(new CargoVisionIOSim(), imuSubsystem);
          driveSubsystem =
              new DriveSubsystem(
                  driverController,
                  imuSubsystem,
                  new WheelIOSim(Corner.FRONT_LEFT),
                  new WheelIOSim(Corner.FRONT_RIGHT),
                  new WheelIOSim(Corner.REAR_LEFT),
                  new WheelIOSim(Corner.REAR_RIGHT));
          break;
        default:
          throw new UnknownTargetRobotException();
      }
    }

    superstructureSubsystem = new SuperstructureSubsystem(swiffer, arm);
    localization = new Localization(driveSubsystem, cargoVisionSubsystem, imuSubsystem);

    // Configure the button bindings. You must call this after the subsystems are defined since they
    // are used to add command requirements.
    configureDriverButtonBindings();
    configureCopilotButtonBindings();

    // TODO: Add autonomous command
    autoCommand = new ParallelCommandGroup();
  }

  private void configureDriverButtonBindings() {
    // Testing PathPlanner
    driverController.bButton.whenHeld(
        new PPCommand(PPPaths.simplePath, driveSubsystem, localization));

    // Testing autonomous
    driverController.yButton.whenHeld(new VelocityControlTestCommand(driveSubsystem));

    // Resetting field oriented control
    driverController.xButton.whenActive(imuSubsystem::zeroHeading);
  }

  private void configureCopilotButtonBindings() {
    // Align for shooting
    copilotController.aButton.whenHeld(new UpperHubAlignCommand(driveSubsystem, localization));

    // Snarfing
    copilotController.rightTrigger.whileHeld(new ArmDownAndSnarfCommand(superstructureSubsystem));

    // Shooting
    copilotController.leftTrigger.whileHeld(
        new ArmUpAndSwifferShootCommand(superstructureSubsystem));
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
