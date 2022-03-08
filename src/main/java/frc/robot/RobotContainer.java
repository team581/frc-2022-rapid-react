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
import frc.robot.fms.FmsIOReal;
import frc.robot.fms.FmsIOReplay;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.*;
import frc.robot.limelight_cargo.CargoLimelightSubsystem;
import frc.robot.limelight_upper.UpperHubLimelightSubsystem;
import frc.robot.misc.exceptions.UnknownTargetRobotException;
import frc.robot.paths.commands.SimplePathCommand;
import frc.robot.superstructure.SuperstructureSubsystem;
import frc.robot.superstructure.commands.LifterDownAndSnarfCommand;
import frc.robot.superstructure.commands.LifterUpAndSwifferShootCommand;
import frc.robot.superstructure.commands.LifterUpAndSwifferStopCommand;
import frc.robot.superstructure.lifter.*;
import frc.robot.superstructure.swiffer.*;
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

  private final FmsSubsystem fmsSubsystem;
  private final ImuSubsystem imuSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final UpperHubLimelightSubsystem upperLimelightSubsystem =
      new UpperHubLimelightSubsystem();
  private final CargoLimelightSubsystem cargoLimelightSubsystem = new CargoLimelightSubsystem();
  private final Swiffer swiffer;
  private final Lifter lifter;
  private final SuperstructureSubsystem superstructureSubsystem;

  private final Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      fmsSubsystem = new FmsSubsystem(new FmsIOReplay());
      lifter = new Lifter(new LifterIOReplay());
      swiffer = new Swiffer(new SwifferIOReplay());
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
          fmsSubsystem = new FmsSubsystem(new FmsIOReal());
          lifter = new Lifter(new LifterIOReal());
          swiffer = new Swiffer(new SwifferIOReal());
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
          fmsSubsystem = new FmsSubsystem(new FmsIOReal());
          lifter = new Lifter(new LifterIOReplay());
          swiffer = new Swiffer(new SwifferIOReplay());
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
          // FMS will work even in simulation
          fmsSubsystem = new FmsSubsystem(new FmsIOReal());
          lifter = new Lifter(new LifterIOSim());
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
        new ParallelCommandGroup(
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
