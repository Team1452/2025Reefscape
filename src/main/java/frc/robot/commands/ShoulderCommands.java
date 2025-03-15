package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shoulder.Shoulder;

public class ShoulderCommands {
  public static Command moveShoulderTo(Shoulder shoulder, double angle) {
    return Commands.run(() -> shoulder.setRAngle(angle), shoulder)
        .alongWith(Commands.print("going to " + angle))
        .until(() -> shoulder.nearRPosition() && shoulder.getSpeed() < 1)
        .andThen(Commands.print("At " + angle));
  }

  public static Command foldIn(Shoulder shoulder) {
    return moveShoulderTo(shoulder, MathUtil.clamp(shoulder.getAngle(), 0.15, 0.35));
  }

  public static Command place(Shoulder shoulder) {
    return moveShoulderTo(shoulder, ElevatorConstants.scoringAngle);
  }
}
