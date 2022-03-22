// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.misc.exceptions.UnknownTargetRobotException;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Change this depending on what your environment is
  public static final Env ENV = Env.DEVELOPMENT;

  /** The roboRIO's serial number. */
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

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

  /** The robot the code should target. */
  public enum TargetRobot {
    /** The 2022 competition robot. */
    COMP_BOT("0305cd6b"),
    /** The 2020 test bot. */
    TEST_2020_BOT("031617f6"),
    /** A simulated robot. */
    SIM_BOT("simbot");

    private final String serialNumber;

    TargetRobot(String serialNumber) {
      this.serialNumber = serialNumber;
    }
  }

  public static TargetRobot getRobot() {
    if (RobotBase.isSimulation()) {
      return TargetRobot.SIM_BOT;
    }

    if (TargetRobot.COMP_BOT.serialNumber.equals(SERIAL_NUMBER)) {
      return TargetRobot.COMP_BOT;
    }

    if (TargetRobot.TEST_2020_BOT.serialNumber.equals(SERIAL_NUMBER)) {
      return TargetRobot.TEST_2020_BOT;
    }

    throw new IllegalStateException("Unknown robot serial number: " + SERIAL_NUMBER);
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case COMP_BOT:
      case TEST_2020_BOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIM_BOT:
        return Mode.SIM;
    }

    throw new UnknownTargetRobotException();
  }

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int COPILOT_CONTROLLER_PORT = 1;

  /** The number of seconds each iteration takes. WPILib default is 20ms. */
  public static final double PERIOD_SECONDS = Units.millisecondsToSeconds(10);

  static {
    System.out.println("SERIAL NUMBER: " + SERIAL_NUMBER);
    System.out.println("RESOLVED ROBOT: " + getRobot());
    System.out.println("MODE: " + getMode());
  }

  private Constants() {}
}
