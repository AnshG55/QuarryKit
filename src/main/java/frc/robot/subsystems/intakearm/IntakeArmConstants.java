package frc.robot.subsystems.intakearm;

public class IntakeArmConstants {
  public static int armMotorCanID = 0;
  public static double armGearRatio = 1;
  public static double armCurrentLimit = 60;
  public static double targetVelocity = 2;
  public static double targetAcceleration = 32;
  public static final double armLengthMeters = 0.5;
  public static final double armMassKg = 2.0;
  public static final double minAngleRad = Math.toRadians(-10);
  public static final double maxAngleRad = Math.toRadians(100);
  public static final double startingAngleRad = Math.toRadians(0);
  public static final double armMomentOfInertia =
      (1.0 / 3.0) * armMassKg * Math.pow(armLengthMeters, 2);
  public static final double kGConstants = 0;
  public static final double kVConstants = 0;
  public static final double kAConstants = 0;
  public static final double kPConstants = 120;
  public static final double kIConstants = 0;
  public static final double kDConstants = 0;
}
