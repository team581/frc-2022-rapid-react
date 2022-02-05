// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerUtil {
  private final XboxController controller;
  // TODO: Fine-tune these values
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(7);

  public ControllerUtil(XboxController controller) {
    this.controller = controller;
  }

  /** Scale a joystick value. */
  private static double joystickScale(double x) {
    return (Math.signum(x) * (Math.pow(x, 2)) / 10);
  }

  /** The rotation across the robot's x-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getXPercentage() {
    return joystickScale(xLimiter.calculate(controller.getRightX()));
  }

  /** The translation across the robot's y-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getYPercentage() {
    return joystickScale(yLimiter.calculate(controller.getLeftY()));
  }

  /** The rotation about the robot's z-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getThetaPercentage() {
    return joystickScale(thetaLimiter.calculate(controller.getLeftX()));
  }
}
