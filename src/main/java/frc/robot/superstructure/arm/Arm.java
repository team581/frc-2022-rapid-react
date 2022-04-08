// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.arm.ArmIO.Inputs;
import frc.robot.superstructure.lights.Lights;
import frc.robot.superstructure.swiffer.Swiffer;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  /** Mass of arm in kilograms. */
  public static final double ARM_MASS = Swiffer.MASS + (4776.833 / 1e3);

  /** Length of arm in meters. */
  public static final double ARM_LENGTH = 838.20 / 1e3;

  /** Moment of inertia (Iyy in Fusion 360) in kg m^2. */
  public static final double MOMENT_OF_INERTIA = 3.084;

  /** Gear ratio of motor. */
  public static final double GEARING = 300.0 / 7.0;

  /** The starting positon of the arm. */
  // Arm starts in the up position
  public static final ArmPosition STARTING_POSITION = ArmPosition.UP;

  private final Lights lights;

  private final ArmIO io;
  private final Inputs inputs = new Inputs();

  private ArmPosition desiredPosition = STARTING_POSITION;
  private double desiredVoltageVolts = 0;

  /** Creates a new Arm. */
  public Arm(ArmIO io, Lights lights) {
    this.io = io;
    this.lights = lights;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);

    if (DriverStation.isEnabled()) {
      doPositionControlLoop();
    }

    final var isAtGoal = atGoal();
    Logger.getInstance().recordOutput("Arm/Goal/Position", desiredPosition.toString());
    Logger.getInstance().recordOutput("Arm/Goal/AtGoal", isAtGoal);
    Logger.getInstance().recordOutput("Arm/Position", getPosition().toString());
    Logger.getInstance().recordOutput("Arm/DesiredVoltageVolts", desiredVoltageVolts);

    lights.setSubsystemState(desiredPosition, isAtGoal);
  }

  /** Set the desired position of the arm. */
  public void setDesiredPosition(ArmPosition position) {
    desiredPosition = position;
  }

  /** Check if the arm is at the provided position. */
  public boolean atPosition(ArmPosition position) {
    return getPosition() == position;
  }

  private void doPositionControlLoop() {
    if (atGoal()) {
      desiredVoltageVolts = 0;
    } else {
      switch (desiredPosition) {
        case UP:
          desiredVoltageVolts = 2.0;
          break;
        case DOWN:
          desiredVoltageVolts = -1.0;
          break;
        default:
          throw new IllegalArgumentException("Invalid desired position");
      }
    }

    io.setVoltage(desiredVoltageVolts);
  }

  private boolean atGoal() {
    return atPosition(desiredPosition);
  }

  private ArmPosition getPosition() {
    if (inputs.upwardLimitSwitchEnabled) {
      if (inputs.downwardLimitSwitchEnabled) {
        // Both limit switches should never be enabled
        return ArmPosition.UNKNOWN;
      }

      return ArmPosition.UP;
    } else if (inputs.downwardLimitSwitchEnabled) {
      return ArmPosition.DOWN;
    } else {
      return ArmPosition.UNKNOWN;
    }
  }
}
