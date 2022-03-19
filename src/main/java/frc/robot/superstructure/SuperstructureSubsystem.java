// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.arm.Arm;
import frc.robot.superstructure.commands.ArmUpAndSwifferStopCommand;
import frc.robot.superstructure.swiffer.Swiffer;

public class SuperstructureSubsystem extends SubsystemBase {
  public final Swiffer swiffer;
  public final Arm arm;

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(Swiffer swiffer, Arm arm) {
    this.swiffer = swiffer;
    this.arm = arm;

    setDefaultCommand(
        new ArmUpAndSwifferStopCommand(this)
            .perpetually()
            .withName("Perpetual" + ArmUpAndSwifferStopCommand.class.getSimpleName()));
  }

  @Override
  public void periodic() {
    swiffer.periodic();
    arm.periodic();
  }
}
