package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  public static class ShoulderIOInputs {
    public double internalAngle;
    public double shoulderAngle;
    public double shoulderSpeed;
  }

  public default void updateInputs(ShoulderIOInputs inputs) {}

  public default void resetIAccum() {}

  public default void setShoulderAngle(double angle) {}
}
