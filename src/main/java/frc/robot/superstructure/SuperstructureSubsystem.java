// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.lifter.Lifter;
import frc.robot.superstructure.swiffer.Swiffer;

public class SuperstructureSubsystem extends SubsystemBase {
  public final Swiffer swiffer;
  public final Lifter lifter;

  /** Creates a new Superstructure. */
  public SuperstructureSubsystem(Swiffer swiffer, Lifter lifter) {
    this.swiffer = swiffer;
    this.lifter = lifter;
  }

  @Override
  public void periodic() {
    swiffer.periodic();
    lifter.periodic();
  }
}
