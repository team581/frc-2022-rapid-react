// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Change this depending on which robot you are targeting for deployment
  private static final TargetRobot TARGET_ROBOT = TargetRobot.SIM_BOT;

  // Change this depending on what your environment is
  public static final Env ENV = Env.DEVELOPMENT;

  /** The robot the code should target. */
  public enum TargetRobot {
    /** The 2022 competition robot. */
    COMP_BOT,
    /** The 2020 test bot. */
    TEST_2020_BOT,
    /** A simulated robot. */
    SIM_BOT
  }

  public enum Env {
    /** Running during developent. This does not include test matches at comps! */
    DEVELOPMENT,
    /** Running in an actual competition. */
    PRODUCTION,
  }

  public enum Mode {
    /** Your robot is actually running in the real world. */
    REAL,
    /** Your robot is running in a simulation using replayed data. */
    REPLAY,
    /** Your robot is running in a simulation. */
    SIM
  }

  public static TargetRobot getRobot() {
    if (RobotBase.isSimulation()) {
      return TargetRobot.SIM_BOT;
    }

    if (TARGET_ROBOT == TargetRobot.SIM_BOT && RobotBase.isReal()) {
      System.out.println("WARNING: You are trying to run a simulation with a real robot!");
    }

    return TARGET_ROBOT;
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case COMP_BOT:
      case TEST_2020_BOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_BOT:
        return Mode.SIM;
    }

    throw new IllegalStateException("Unknown target robot");
  }

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int COPILOT_CONTROLLER_PORT = 1;

  public static final double LOOP_TIME_SECONDS = 0.02;

  static {
    System.out.println("SPECIFIED ROBOT: " + TARGET_ROBOT);
    System.out.println("RESOLVED ROBOT: " + getRobot());
    System.out.println("MODE: " + getMode());
  }

  private Constants() {}
}
