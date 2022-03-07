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
    // Values taken from https://gamepad-tester.com/
    aButton = new JoystickButton(controller, 1);
    bButton = new JoystickButton(controller, 2);
    xButton = new JoystickButton(controller, 3);
    yButton = new JoystickButton(controller, 4);

    leftTrigger = new JoystickButton(controller, 7);
    rightTrigger = new JoystickButton(controller, 8);

    leftBumper = new JoystickButton(controller, 5);
    rightBumper = new JoystickButton(controller, 6);

    back = new JoystickButton(controller, 9);
    start = new JoystickButton(controller, 10);

    leftStick = new JoystickButton(controller, 11);
    rightStick = new JoystickButton(controller, 12);
  }

  public final Button aButton;
  public final Button bButton;
  public final Button xButton;
  public final Button yButton;

  public final Button leftTrigger;
  public final Button rightTrigger;

  public final Button leftBumper;
  public final Button rightBumper;

  public final Button leftStick;
  public final Button rightStick;

  public final Button back;
  public final Button start;
}
