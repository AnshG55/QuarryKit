package frc.robot.subsystems.intakearm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
  @AutoLog
  public static class IntakeArmIOInputs {
    public double IntakeArmPositionRad = 0.0;
    public double IntakeArmVelocityRadPerSec = 0.0;
    public double IntakeArmAppliedVolts = 0.0;
    public double IntakeArmCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeArmIOInputs inputs) {}

  public default void setIntakeArmVoltage(double volts) {}

  public default void setIntakeArmPosition(double rads) {}

  public default void updateIntakeArmGains() {}
}
