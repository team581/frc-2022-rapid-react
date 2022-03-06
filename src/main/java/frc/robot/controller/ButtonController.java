// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** A wrapper around {@link XboxController} for triggering commands using buttons. */
public class ButtonController {
  public ButtonController(XboxController controller) {
    aButton = new JoystickButton(controller, XboxController.Button.kA.value);
    bButton = new JoystickButton(controller, XboxController.Button.kB.value);
    xButton = new JoystickButton(controller, XboxController.Button.kX.value);
    yButton = new JoystickButton(controller, XboxController.Button.kY.value);

    leftTrigger = new JoystickButton(controller, XboxController.Axis.kLeftTrigger.value);
    rightTrigger = new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);

    leftBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  }

  public final Button aButton;
  public final Button bButton;
  public final Button xButton;
  public final Button yButton;

  public final Button leftTrigger;
  public final Button rightTrigger;

  public final Button leftBumper;
  public final Button rightBumper;
}
