package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int kmotorOnePort = 8;
  public static final int kmotorTwoPort = 9;
  public static final int kshoulderPort = 10;
  public static final boolean motorsInverted = true;
  public static final double[] kElevatorHeights = {
    28.5, 0, 0, 17, 42
  }; // Handoff height, tier 1, 2, 3, 4.
  // Handoff height is also the minimum height that the elevator can be at when the shoulder is down
  // and the intake is in.
  public static final double scoringAngle =
      0.37; // In rotations of the shoulder, this is the angle that the shoulder should be at when
  // scoring.
  public static final double[] kElevatorGains = {
    0.2, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double[] kShoulderGains = {
    2, // P
    0.02, // I
    2.5, // D
    0.1 // F
  };
  public static final double kElevatorTolerance =
      0.25; // For the (isNear) method, in rotations11   of the motor.
  public static final double shoulderLength =
      19.5; // The minimum height that the elevator can be at when the shoulder directly down
  public static final double kShoulderOffset = 0.034 + 0.008; // In rotations of the shoulder.
  public static final double maxHeight = 43; // In rotations of the motor.
  public static final double intakeHeight =
      10; // In rotations of the motor. This is the minimum height that the elevator can be at when
  // the intake is in. (and the shoulder is up)
  public static final int klimitSwitchPort = 1; // DIO port one.
  public static final double kShoulderConversionFactor =
      1.0 / 45; // From the internal encoder to rotations of the shoulder.
}
