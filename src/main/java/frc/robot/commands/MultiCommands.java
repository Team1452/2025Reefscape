package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shoulder.Shoulder;

public class MultiCommands {
  private MultiCommands() {}

  public static Command maintainAngles(Intake intake, Elevator elevator, Shoulder shoulder) {
    return Commands.run(
            () -> {
              intake.setIntakeAngle(intake.getIntakeAngle());
              elevator.setRHeight(elevator.getHeight());
              shoulder.setRAngle(shoulder.getAngle());
            })
        .ignoringDisable(true);
  }

  public static Command handOff(Intake intake, Elevator elevator, Shoulder shoulder) {
    return Commands.sequence(
        Commands.parallel(
            ElevatorCommands.moveElevatorTo(
                elevator, ElevatorConstants.kElevatorHeights[0] + 10), // move the elevator up.
            Commands.waitUntil(() -> elevator.getHeight() > ElevatorConstants.kElevatorHeights[0])
                .andThen(
                    ShoulderCommands.moveShoulderTo(
                        shoulder,
                        0.75)) //// Wait until the elevator is high enough up to start rotating the
            // arm down.
            ),
        Commands.parallel(
            ElevatorCommands.moveElevatorTo(
                elevator,
                ElevatorConstants.kElevatorHeights[0]), // Move the elevator down to handoff height.
            IntakeCommands.spitOut(
                intake, true) // When the elevator is less than 3 rotations away from being at the
            // handoff, bubble up the coral.
            ),
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[0]
                + 8), // Move the elevator up slightly so we can rotate the shoulder.
        ShoulderCommands.moveShoulderTo(shoulder, 0.25), // Move the shoulder up
        IntakeCommands.moveIntakeTo(
            intake, IntakeConstants.intakeLevelOneAngle), // Move the intake out of the way.
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[1]
                + 5) // Move the elevator back down to 0. (Triggers should handle the collisions and
        // automatically move the intake out of the way)
        );
  }
}
