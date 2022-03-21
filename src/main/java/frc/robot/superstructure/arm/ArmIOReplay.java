// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure.arm;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.Constants;

public class ArmIOReplay implements ArmIO {
  @Override
  public LinearSystem<N2, N1, N1> getPlant() {
    switch (Constants.getRobot()) {
      case SIM_BOT:
      default:
        return LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNEO(2), Arm.MOMENT_OF_INERTIA, Arm.GEARING);
    }
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Intentionally left empty
  }

  @Override
  public void setVoltage(double volts) {
    // Intentionally left empty
  }
}
