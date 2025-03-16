package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int IntakeID = 50;
  public static final int RollerID = 11;
  public static final double intakeSuckSpeed = 0.35;
  public static final double intakeL1Speed = -0.15;
  public static final double intakeHandOffAngle = -2.8;
  public static final double intakeIntakeAngle = 28.5;
  public static final double intakeLevelOneAngle = 10;
  public static final double intakeStartUpAngle = 2.93;

  public static final boolean reversedRotator = true;
  public static final boolean reversedSucker = true;
  public static final double[] kIntakeGains = {
    1, // P
    0, // I
    0, // D
    0.2 // F
  };
  public static final double intakeRotateOutSpeed = 0.4;
  public static final double intakeRotateInSpeed = -0.4;
}
