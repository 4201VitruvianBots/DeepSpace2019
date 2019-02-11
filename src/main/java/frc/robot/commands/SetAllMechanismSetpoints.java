/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.wrist.SetWristSetpoint;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetAllMechanismSetpoints extends CommandGroup {
    public SetAllMechanismSetpoints(int position) {
        // Set the position of the elevator
        switch(position) {
            case 4: // Rocket Level 3 Scoring Positions
                if(Intake.outtakeState == 2) {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_CARGO_HIGH_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_HIGH_POSITION));
                } else {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_HIGH_POSITION));
                }
            case 3: // Rocket Level 2 Scoring Positions
                if(Intake.outtakeState == 2) {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_EXTENDED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_MID_POSITION));
                } else {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_MID_POSITION));
                }
                break;
            case 2: // Rocket Level 1 Scoring Positions
                if(Intake.outtakeState == 2) {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_EXTENDED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_LOW_POSITION));
                } else {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_LOW_POSITION));
                }
                break;
            case 1: // Cargo Ship Scoring Positions
                if(Intake.outtakeState == 2) {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_EXTENDED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CARGO_SHIP_POSITION));
                } else {
                    addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
                    addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_HATCH_LOW_POSITION));
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
