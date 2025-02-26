package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;


import java.util.function.DoubleSupplier;


public class ElevatorCommands {

    public static Command controlElevatorWithController(DoubleSupplier inputx, DoubleSupplier inputy, Elevator elevator) {//Control the elevator, this will only alter the RHeight and RAngle, moving of motors is handled in Elevator.
        return Commands.run(()-> {
            elevator.setRHeight(MathUtil.applyDeadband(inputy.getAsDouble(), 0.3)/ 3.5); //Divide by 3.5 to adjust stepSize.
            elevator.setRAngle(MathUtil.applyDeadband(inputx.getAsDouble(), 0.2)/ 5);
        });
    }

    public static InstantCommand goToTier(int tier, Elevator elevator) {
        return new InstantCommand(
            ()-> elevator.setRHeight(ElevatorConstants.kElevatorHeights[tier]) //Set the height to the height based on a list of tier heights.
        , elevator);
    }
    public static InstantCommand placeOnTier(int tier, Elevator elevator) {
        //Assuming alignment to reef.
        return new InstantCommand(
            ()-> {
                elevator.setRAngle(ElevatorConstants.scoringAngle);
                elevator.setRHeight(ElevatorConstants.kElevatorHeights[tier]); 
            } ,elevator);
    }
    public static Command pickUpCoralFromIntake(Elevator elevator) {
        return Commands.sequence( 
            Commands.run(()->{
                elevator.setRAngle(0.75); //Move the arm down.
                elevator.setRHeight(ElevatorConstants.kElevatorHeights[0]+3); //Move the elevator to above the handoff height.
            },elevator),
            Commands.waitUntil(()->MathUtil.isNear(0.75, elevator.getShoulderAngle(), 0.01)), //Wait until the arm is down,
            goToTier(0,elevator) //Then move down to the coral. to intake it.
            );
    }
  }
    
