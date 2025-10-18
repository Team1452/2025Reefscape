package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;
  private static double elevatorRHeight = 0;

  // Inputs from the elevator hardware.
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /**
   * @param io The interfce)
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setHeight(elevatorRHeight);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/ElevatorRHeight", elevatorRHeight);
    Logger.recordOutput("Elevator/ElevatorHeight", inputs.height);
    Commands.sequence(Commands.print(Double.toString(getHeight())));
  }

  public boolean nearRPosition() {
    return MathUtil.isNear(elevatorRHeight, inputs.height, ElevatorConstants.kElevatorTolerance);
  }

  public boolean nearPosition(double height) {
    return MathUtil.isNear(height, inputs.height, ElevatorConstants.kElevatorTolerance);
  }

  public void setMotorSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void adjustRHeight(double height) {
    elevatorRHeight += height;
    System.out.println(getHeight());
  }

  public void setRHeight(double height) {
    elevatorRHeight = height;
    System.out.println(getRHeight());
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public boolean eLimitSwitch() {
    return inputs.elevatorlimitSwtich;
  }

  public double getHeight() {
    return inputs.height;
  }

  public double getRHeight() {
    return elevatorRHeight;
  }
}
