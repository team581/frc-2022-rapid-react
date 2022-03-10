// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import org.littletonrobotics.junction.inputs.LoggableInputs;

/** A subsystem's IO interface. */
public interface SubsystemIO<T extends LoggableInputs> {
  /** Updates the set of loggable inputs. */
  public void updateInputs(T inputs);
}
