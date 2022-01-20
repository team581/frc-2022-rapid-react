// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups.vision;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.WaitForVisionTargetCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.RumblePattern;
import frc.robot.vision.Vision;

public class UpperHubAlignCommand extends SequentialCommandGroup {
  public UpperHubAlignCommand(
      Vision vision, LimelightSubsystem limelight, XboxController controller) {
    addCommands(
        // Enable vision
        new InstantCommand(() -> vision.useVisionTarget(vision.upperHub)),

        // Wait for driver to point the Limelight at the upper hub while rumbling the controller
        race(
            new WaitForVisionTargetCommand(limelight),
            // Rumble the controller indefinitely, this is a parallel race so the other command will
            // interrupt this one
            new RumbleCommand(controller, new RumblePattern(1, 0.15, 0.15))),

        // TODO: Create AlignWithTargetCommand which accepts a VisionTarget
        // TODO: Handle losing sight of target in AlignWithTargetCommand

        // Rumble until the command is interrupted by the driver
        new RumbleCommand(controller, new RumblePattern(1, 0.25, 0.15, 5)));
  }
}
