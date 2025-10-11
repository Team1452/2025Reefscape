package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shoulder.Shoulder;
import java.util.function.DoubleSupplier;

public class MultiCommands {
  private MultiCommands() {}

  public static Command startUpAngles(Elevator elevator, Shoulder shoulder) {
    return Commands.run(
            () -> {
              elevator.setRHeight(elevator.getHeight());
              shoulder.setRAngle(shoulder.getAngle());
            })
        .ignoringDisable(true);
  }

  public static Command killAllComands(Elevator elevator, Shoulder shoulder) {
    return (Commands.idle(elevator, shoulder));
  }

  public static Command handOff(
      Elevator elevator, Shoulder shoulder, DoubleSupplier shoulderAngleHandoff) {

    return Commands.sequence(
        Commands.parallel(
            Commands.print(shoulderAngleHandoff.getAsDouble() + ""),
            ElevatorCommands.moveElevatorTo(
                elevator, ElevatorConstants.kElevatorHeights[0] + 10), // move the elevator up.
            Commands.waitUntil(() -> elevator.getHeight() > ElevatorConstants.kElevatorHeights[0])
                .andThen(
                    ShoulderCommands.moveShoulderTo(
                        shoulder,
                        shoulderAngleHandoff)) //// Wait until the elevator is high enough up to
            ),
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[0]), // Move the elevator down to handoff height.
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[0]
                + 8), // Move the elevator up slightly so we can rotate the shoulder.
        ShoulderCommands.moveShoulderTo(shoulder, 0.25), // Move the shoulder up
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[1]
                + 5) // Move the elevator back down to 0. (Triggers should handle the collisions
        // and
        // automatically move the intake out of the way)
        );
  }

  public static Command safeMode(Elevator elevator, Shoulder shoulder) {
    double returnPositions[] = {elevator.getRHeight(), shoulder.getRAngle()};
    return Commands.run(
            () -> {
              elevator.setRHeight(Math.min(returnPositions[0], 5));
              shoulder.setRAngle(0.25);
            },
            elevator,
            shoulder)
        .finallyDo(
            () -> {
              elevator.setRHeight(returnPositions[0]);
              shoulder.setRAngle(returnPositions[1]);
            });
  }
}
