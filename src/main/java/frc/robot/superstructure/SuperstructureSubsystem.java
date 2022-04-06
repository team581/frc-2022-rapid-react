// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.arm.Arm;
import frc.robot.superstructure.commands.ArmUpAndStopCommand;
import frc.robot.superstructure.lights.Lights;
import frc.robot.superstructure.swiffer.Swiffer;

public class SuperstructureSubsystem extends SubsystemBase {
  public final Swiffer swiffer;
  public final Arm arm;
  public final Lights lights;

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(Swiffer swiffer, Arm arm, Lights lights) {
    this.swiffer = swiffer;
    this.arm = arm;
    this.lights = lights;

    setDefaultCommand(
        new ArmUpAndStopCommand(this)
            .perpetually()
            .withName("Perpetual" + ArmUpAndStopCommand.class.getSimpleName()));
  }

  @Override
  public void periodic() {
    swiffer.periodic();
    arm.periodic();
  }
}
