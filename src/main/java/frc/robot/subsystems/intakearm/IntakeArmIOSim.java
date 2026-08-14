package frc.robot.subsystems.intakearm;

import static frc.robot.subsystems.intakearm.IntakeArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeArmIOSim implements IntakeArmIO {
  private DCMotorSim armSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, armGearRatio),
          DCMotor.getCIM(1));

  private double armAppliedVolts = 0;

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    armSim.setInputVoltage(armAppliedVolts);
    armSim.update(0.02);

    inputs.IntakeArmPositionRad = armSim.getAngularPositionRad();
    inputs.IntakeArmVelocityRadPerSec = armSim.getAngularVelocityRadPerSec();
    inputs.IntakeArmAppliedVolts = armAppliedVolts;
    inputs.IntakeArmCurrentAmps = armSim.getCurrentDrawAmps();
  }

  @Override
  public void setIntakeArmVoltage(double volts) {
    armAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
