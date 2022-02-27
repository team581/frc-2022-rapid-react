// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** The robot the code should target. */
  public enum TargetRobot {
    /** The 2022 competition robot. */
    COMP_BOT,
    /** The 2020 test bot. */
    TEST_2020_BOT,
  }

  public enum Env {
    /** Running during developent. This does not include test matches at comps! */
    DEVELOPMENT,
    /** Running in an actual competition.. */
    PRODUCTION,
  }

  // Change this depending on which robot you are targeting for deployment
  public static final TargetRobot TARGET_ROBOT = TargetRobot.TEST_2020_BOT;

  // Change this depending on what your environment is
  public static final Env ENV = Env.DEVELOPMENT;

  public static final int CONTROLLER_PORT = 0;

  private Constants() {}
}
