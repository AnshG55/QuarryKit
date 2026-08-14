// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakearm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
  /** Creates a new IntakeArm. */
  private final IntakeArmIO io;

  private final IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

  public IntakeArm(IntakeArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("IntakeArm", inputs);
  }

  public Command IntakeArmUp() {
    return run(() -> {
          io.setIntakeArmVoltage(1);
        })
        .finallyDo(
            () -> {
              io.setIntakeArmVoltage(0.1);
            });
  }

  public Command IntakeArmtoSetPoint(double rads) {
    return run(() -> {
          io.setIntakeArmPosition(rads);
        })
        .until(() -> Math.abs(inputs.IntakeArmPositionRad - rads) < 0.02)
        .finallyDo(
            () -> {
              io.setIntakeArmVoltage(0);
            });
  }
}
