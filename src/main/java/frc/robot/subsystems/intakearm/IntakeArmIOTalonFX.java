package frc.robot.subsystems.intakearm;

import static frc.robot.subsystems.intakearm.IntakeArmConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeArmIOTalonFX implements IntakeArmIO {

  private final TalonFX armMotor = new TalonFX(armMotorCanID);

  private final StatusSignal<Angle> armPositionRot = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> armVelocityRotPerSec = armMotor.getVelocity();
  private final StatusSignal<Voltage> armAppliedVolts = armMotor.getMotorVoltage();
  private final StatusSignal<Current> armCurrentAmps = armMotor.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage magicRequest;

  private final Slot0Configs configSlot;
  private LoggedNetworkNumber kG = new LoggedNetworkNumber("Intake Arm kG", kGConstants);
  private LoggedNetworkNumber kV = new LoggedNetworkNumber("Intake Arm kV", kVConstants);
  private LoggedNetworkNumber kA = new LoggedNetworkNumber("Intake Arm kA", kAConstants);
  private LoggedNetworkNumber kP = new LoggedNetworkNumber("Intake Arm kP", kPConstants);
  private LoggedNetworkNumber kI = new LoggedNetworkNumber("Intake Arm kI", kIConstants);
  private LoggedNetworkNumber kD = new LoggedNetworkNumber("Intake Arm kD", kDConstants);

  public IntakeArmIOTalonFX() {
    configSlot = new Slot0Configs();
    configSlot.kG = kG.get();
    configSlot.kV = kV.get();
    configSlot.kA = kA.get();
    configSlot.kP = kP.get();
    configSlot.kI = kI.get();
    configSlot.kD = kD.get();
    configSlot.GravityType = GravityTypeValue.Arm_Cosine;
    configSlot.GravityArmPositionOffset = 0;

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.CurrentLimits.SupplyCurrentLimit = armCurrentLimit;
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    armConfig.Feedback.SensorToMechanismRatio = armGearRatio;
    armConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(targetVelocity)
            .withMotionMagicAcceleration(targetAcceleration);
    armConfig.Slot0 = configSlot;

    tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, armPositionRot, armVelocityRotPerSec, armAppliedVolts, armCurrentAmps);

    magicRequest = new MotionMagicVoltage(0);
    armMotor.setPosition(startingAngleRad);
  }

  @Override
  public void updateInputs(IntakeArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        armPositionRot, armVelocityRotPerSec, armAppliedVolts, armCurrentAmps);

    inputs.IntakeArmPositionRad = Units.rotationsToRadians(armPositionRot.getValueAsDouble());
    inputs.IntakeArmVelocityRadPerSec =
        Units.rotationsToRadians(armVelocityRotPerSec.getValueAsDouble());
    inputs.IntakeArmAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.IntakeArmCurrentAmps = armCurrentAmps.getValueAsDouble();

    Logger.recordOutput(
        "Intake Arm Profile Position rads/s",
        Units.rotationsToRadians(armMotor.getClosedLoopReference().getValueAsDouble()));
    Logger.recordOutput(
        "Intake Arm Profile Velocity rads/s",
        Units.rotationsToRadians(armMotor.getClosedLoopReferenceSlope().getValueAsDouble()));

    Logger.recordOutput("Intake Arm kG", kG.get());
    Logger.recordOutput("Intake Arm kV", kV.get());
    Logger.recordOutput("Intake Arm kA", kA.get());
    Logger.recordOutput("Intake Arm kP", kP.get());
    Logger.recordOutput("Intake Arm kI", kI.get());
    Logger.recordOutput("Intake Arm kD", kD.get());
  }

  @Override
  public void setIntakeArmVoltage(double volts) {
    armMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setIntakeArmPosition(double rads) {
    armMotor.setControl(magicRequest.withPosition(rads));
  }

  @Override
  public void updateIntakeArmGains() {
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    armConfig.Slot0.kP = kP.get();
    armConfig.Slot0.kI = kI.get();
    armConfig.Slot0.kD = kD.get();

    armConfig.Slot0.kS = kG.get();
    armConfig.Slot0.kV = kV.get();
    armConfig.Slot0.kA = kA.get();

    tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));
  }
}
