package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ShoulderIO io;
  private static double shoulderRAngle = 0.29;
  private double shoulderHandoffAngle;

  // Inputs from the elevator hardware.
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
  /**
   * @param io The interfce)
   */
  public Shoulder(ShoulderIO io) {
    this.io = io;
    shoulderHandoffAngle = 0.75;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Refresh the inputs.
    if (Math.abs(shoulderRAngle - inputs.shoulderAngle) > 0.1) {
      io.resetIAccum();
    }
    io.setShoulderAngle(shoulderRAngle);
    Logger.processInputs("ElevatorShoulder", inputs);
    Logger.recordOutput("ElevatorShoulder/AbsShoulderAngle", inputs.shoulderAngle);
    Logger.recordOutput("ElevatorShoulder/InternalShoulderAngle", inputs.internalAngle);
    Logger.recordOutput("ElevatorShoulder/ShoulderSpeed", inputs.shoulderSpeed);
    Logger.recordOutput("ElevatorShoulder/ShoulderRAngle", shoulderRAngle);
  }

  public void setRAngle(double angle) {
    shoulderRAngle = angle;
  }

  public double getAngle() {
    return inputs.shoulderAngle;
  }

  public double getSpeed() {
    return inputs.shoulderSpeed;
  }

  public double getRAngle() {
    return shoulderRAngle;
  }

  public void setPIDFGains(double p, double i, double d, double f) {
    io.setPIDFGains(p, i, d, f);
  }

  public void setShoulderAngleForHandoff(double a) {
    shoulderHandoffAngle = a;
    System.out.println(shoulderHandoffAngle);
  }

  public double getShoulderAngleForHandoff() {
    System.out.println(shoulderHandoffAngle);
    return shoulderHandoffAngle;
  }

  public boolean nearRPosition() {
    return MathUtil.isNear(shoulderRAngle, inputs.shoulderAngle, 0.015);
  }

  public void adjustRAngle(double angle) {
    shoulderRAngle += angle;
  }
}
