// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// github push test

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.MultiCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalons;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final Shoulder shoulder;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandGenericHID fightBox = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final GenericEntry pEntry, iEntry, dEntry, fEntry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalons(TunerConstants.FrontLeft),
                new ModuleIOTalons(TunerConstants.FrontRight),
                new ModuleIOTalons(TunerConstants.BackLeft),
                new ModuleIOTalons(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation),
                new VisionIOLimelight(camera3Name, drive::getRotation),
                new VisionIOLimelight(camera4Name, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));
        break;

      case TEST: // Make a "blank" drive and "blank" vision, as these subsystems are most expensive
        // and not needed for testing.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    elevator = new Elevator(new ElevatorIOSpark());
    shoulder = new Shoulder(new ShoulderIOSpark());
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addDefaultOption("Taxi back", new PathPlannerAuto("LeaveAuto"));
    autoChooser.addOption("Middle Auto", new PathPlannerAuto("MiddleAuto"));

    var tuningTab = Shuffleboard.getTab("PID Tuning");
    // Create slider widgets for P, I, and D.
    pEntry =
        tuningTab
            .add("P Gain", ElevatorConstants.kShoulderGains[0])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();
    iEntry =
        tuningTab
            .add("I Gain", ElevatorConstants.kShoulderGains[1])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .getEntry();
    dEntry =
        tuningTab
            .add("D Gain", ElevatorConstants.kShoulderGains[2])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 5))
            .getEntry();
    fEntry =
        tuningTab
            .add("FF Gain", ElevatorConstants.kShoulderGains[3])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 5))
            .getEntry();

    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemLogic();
  }

  private void configureSubsystemLogic() {
    Trigger elevatorLimitSwtichTrigger = new Trigger(() -> elevator.eLimitSwitch());

    // if the shoulder is down, and the ACTUAL HEIGHT of the elevator is too low, then we neexd to
    // move the shoulder up.
    Trigger shoulderCrashTrigger =
        new Trigger(
            () ->
                shoulder.getAngle() < 1
                    && elevator.getHeight()
                        < (ElevatorConstants.shoulderLength)
                            * Math.cos(Math.PI * 2 * (shoulder.getAngle() - 0.75)));

    // If the REQUESTED HEIGHT of the elevator is lower than a height where it would hit the intake,
    // then we need to move the intake out of the way (if its ACTUALLY IN)

    // If the REQUESTED ANGLE of the intake is in, and the ACTUAL HEIGHT of the elevator is too low,
    // then we need to move the elevator up out of the way.

    shoulderCrashTrigger.onTrue(
        new InstantCommand(() -> shoulder.setRAngle(0.25), shoulder)
            .andThen(
                Commands.print(
                    "Shoulder Crash Trigger"))); // move the shoulder straight up to avoid crashing
    // into the robot.

    elevatorLimitSwtichTrigger.onTrue(
        new InstantCommand(elevator::resetEncoder)
            .andThen(Commands.print("Elevator Limit Switch Trigger"))); // reset the encoder.
  }

  private void configureButtonBindings() {

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Intake and handoff on bumper press.
    fightBox.button(7).toggleOnTrue(MultiCommands.safeMode(elevator, shoulder));

    fightBox
        .button(4)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createBottomLeftRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(150))))));
    fightBox
        .button(6)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createMiddleLeftRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(90))))));
    fightBox
        .button(5)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createTopLeftRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(30))))));
    fightBox
        .button(2)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createBottomLeftRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(-150))))));
    fightBox
        .axisGreaterThan(3, 0.6)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createBottomRightRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(-90))))));
    fightBox
        .axisGreaterThan(2, 0.6)
        .onTrue(
            Commands.sequence(
                AutoBuilder.followPath(
                    new PathPlannerPath(
                        createTopRightRedWaypoints(),
                        constraints,
                        null,
                        new GoalEndState(0.0, Rotation2d.fromDegrees(-30))))));

    fightBox
        .button(10)
        .onTrue(
            new InstantCommand(
                () ->
                    shoulder.setPIDFGains(
                        pEntry.getDouble(ElevatorConstants.kShoulderGains[0]),
                        iEntry.getDouble(ElevatorConstants.kShoulderGains[1]),
                        dEntry.getDouble(ElevatorConstants.kShoulderGains[2]),
                        fEntry.getDouble(ElevatorConstants.kShoulderGains[3])),
                shoulder));

    fightBox.pov(0).onTrue(ElevatorCommands.goToTier(elevator, 2));
    fightBox.button(1).onTrue(ElevatorCommands.goToTier(elevator, 3));
    fightBox.button(3).onTrue(ElevatorCommands.goToTier(elevator, 4));

    controller
        .pov(90)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI))),
                    drive)
                .ignoringDisable(true));

    controller
        .rightBumper()
        .onTrue(MultiCommands.handOff(elevator, shoulder, shoulder::getShoulderAngleForHandoff));

    controller.y().whileTrue(Commands.run(() -> elevator.adjustRHeight(0.5), elevator));
    controller.a().whileTrue(Commands.run(() -> elevator.adjustRHeight(-0.5), elevator));
    controller
        .rightTrigger(0.1)
        .whileTrue(
            Commands.run(
                () -> shoulder.adjustRAngle(0.01 * controller.getRightTriggerAxis()), shoulder));
    controller
        .leftTrigger(0.1)
        .whileTrue(
            Commands.run(
                () -> shoulder.adjustRAngle(-0.01 * controller.getLeftTriggerAxis()), shoulder));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(autoChooser.get());
  }

  public List<Waypoint> createTopRightRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(5.165, 5.165, Rotation2d.fromDegrees(0)));
  }

  public List<Waypoint> createMiddleRightRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(5.902, 4.018, Rotation2d.fromDegrees(0)));
  }

  public List<Waypoint> createBottomRightRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(5.167, 2.879, Rotation2d.fromDegrees(0)));
  }

  public List<Waypoint> createTopLeftRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(3.819, 5.182, Rotation2d.fromDegrees(0)));
  }

  public List<Waypoint> createMiddleLeftRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(3.164, 4, Rotation2d.fromDegrees(0)));
  }

  public List<Waypoint> createBottomLeftRedWaypoints() {
    return PathPlannerPath.waypointsFromPoses(
        drive.getPose(), new Pose2d(3.182, 2.876, Rotation2d.fromDegrees(0)));
  }

}
