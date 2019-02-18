/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.wrist.SetWristSetpoint;
import frc.robot.subsystems.Intake;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetAllMechanismSetpointsOld extends CommandGroup {
    int intakeState = 0;
    @Override
    protected void execute() {
        this.intakeState = Intake.intakeState;
    }

    public SetAllMechanismSetpointsOld(int position) {
        // Set the mechanismPositions of the elevator
        switch (position) {
            case 4: // Rocket Level 3 Scoring Positions
                switch (intakeState) {
                    case 2:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_CARGO_HIGH_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_HIGH_POSITION));
                        break;
                    case 1:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_HATCH_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_GROUND_HIGH_POSITION));
                        break;
                    case 0:
                    default:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_HIGH_POSITION));
                        break;
                }
                break;
            case 3: // Rocket Level 2 Scoring Positions
                switch (intakeState) {
                    case 2:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_CARGO_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_MID_POSITION));
                        break;
                    case 1:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_HATCH_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_GROUND_MID_POSITION));
                        break;
                    case 0:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_MID_POSITION));
                        break;
                }
                break;
            case 2: // Rocket Level 1 Scoring Positions
                switch (intakeState) {
                    case 2:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_CARGO_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_LOW_POSITION));
                        break;
                    case 1:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_HATCH_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_GROUND_LOW_POSITION));
                        break;
                    case 0:
                    default:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_LOW_POSITION));
                        break;
                }
                break;
            case 1: // Cargo Ship Scoring Positions\
                switch (intakeState) {
                    case 2:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_CARGO_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_SHIP_POSITION));
                        break;
                    case 1:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_HATCH_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_GROUND_LOW_POSITION));
                        break;
                    case 0:
                    default:
                        addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_LOW_POSITION));
                        break;
                }
                break;
            case 0: // Elevator, Wrist Home Position
            default:
                addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HOME_POSITION));
                break;
        }
    }
}
