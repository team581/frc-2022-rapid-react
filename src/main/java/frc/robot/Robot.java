// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  public Robot() {
    super(Constants.PERIOD_SECONDS);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    final var isReplay = Constants.getMode() == Constants.Mode.REPLAY;

    // Run as fast as possible during replay
    // setUseTiming(!isReplay);
    // Log & replay "SmartDashboard" values (no tables are logged by default).
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");
    Logger.getInstance().recordMetadata("ProjectName", "RapidReact");
    Logger.getInstance().recordMetadata("TargetRobot", Constants.getRobot().toString());
    Logger.getInstance().recordMetadata("Env", Constants.ENV.toString());
    Logger.getInstance().recordMetadata("Mode", Constants.getMode().toString());
    Logger.getInstance().recordMetadata("PeriodSeconds", Double.toString(Constants.PERIOD_SECONDS));

    ByteLogReceiver receiver;

    if (isReplay) {
      // Prompt the user for a file path on the command line
      System.out.println(
          "If prompted, please enter the filename of the .rlog file to use as a replay");
      final String path = "logs/Log_22-03-24_17-35-39.rlog";
      System.out.println("PATH: " + path);
      // Read log file for replay
      Logger.getInstance().setReplaySource(new ByteLogReplay(path));
      // Save replay results to a new log with the "_sim" suffix
      receiver = new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"));
    } else {
      // Log to USB stick (name will be selected automatically)
      receiver =
          new ByteLogReceiver(Constants.getMode() == Constants.Mode.SIM ? "./" : "/media/sda1/");
    }

    Logger.getInstance().addDataReceiver(receiver);

    if (Constants.ENV == Constants.Env.DEVELOPMENT) {
      // Provide log data over the network, viewable in Advantage Scope.
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    Logger.getInstance().start();

    // Instantiate our RobotContainer. This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    if (Constants.ENV == Constants.Env.DEVELOPMENT) {
      // This pushes lots of data to NetworkTables and can cause lag or network congestion
      setNetworkTablesFlushEnabled(true);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
