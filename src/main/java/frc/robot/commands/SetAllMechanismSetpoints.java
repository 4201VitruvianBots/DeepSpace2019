/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ELEVATOR_SETPOINTS;
import frc.robot.RobotMap.INTAKE_SPEED;
import frc.robot.RobotMap.WRIST_SETPOINTS;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetAllMechanismSetpoints extends InstantCommand {
    public int position;

    public SetAllMechanismSetpoints(int position) {
        requires(Robot.elevator);
        requires(Robot.wrist);
//        requires(Robot.intake);
        this.position = position;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        OI.positionIndex = position;
        // Set the mechanismPositions of the elevator

        switch (position) {
            case 5: // Rocket Level 3 Scoring Positions
                switch (Intake.intakeState) {
                    case 2:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.CARGO);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.CARGO_HIGH);
                        break;
                    case 1:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.HATCH);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_GROUND_HIGH);
                        break;
                    case 0:
                    default:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.RETRACTED);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_HIGH);
                        break;
                }
                break;
            case 4: // Rocket Level 2 Scoring Positions
                switch (Intake.intakeState) {
                    case 2:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.CARGO);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.CARGO_MID);
                        break;
                    case 1:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.HATCH);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_GROUND_MID);
                        break;
                    case 0:
                    default:
                        Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.RETRACTED);
                        Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_HIGH);
                        break;
                }
                break;
            case 3: // Rocket Level 1 Scoring Positions
                switch (Intake.intakeState) {
	                case 2:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.CARGO);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.CARGO_LOW);
	                    break;
	                case 1:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.HATCH_LOW);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_GROUND_LOW);
	                    break;
	                case 0:
	                default:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.RETRACTED);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_LOW);
	                    break;
                }
                break;
            case 2: // Cargo Ship Scoring Positions\
                switch (Intake.intakeState) {
                    case 2:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.CARGO);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.CARGO_SHIP);
                        break;
                    case 1:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.HATCH_LOW);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_GROUND_LOW);
                        break;
                    case 0:
                    default:
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.RETRACTED);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HATCH_LOW);
                        break;
                }
                break;
            case 1: // Intake Positions
                switch (Intake.intakeState) {
                    case 2:
//                        Robot.wrist.setAbsoluteAngle(RobotMap.WRIST_CARGO_INTAKE_STATION_ANGLE);
//                        Robot.elevator.setAbsoluteHeight(RobotMap.ELEVATOR_CARGO_INTAKE_STATION_POSITION);
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.EXTENDED);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HOME);
//                        Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_INTAKE_SPEED);
                        break;
                    case 1:
                        //Robot.wrist.setAbsoluteAngle(RobotMap.WRIST_EXTENDED_ANGLE);
                        //Robot.elevator.setAbsoluteHeight(RobotMap.ELEVATOR_HOME_POSITION);
                        break;
                    case 0:
                    default:
                        //Robot.wrist.setAbsoluteAngle(RobotMap.WRIST_RETRACTED_ANGLE);
                        //Robot.elevator.setAbsoluteHeight(RobotMap.ELEVATOR_HATCH_INTAKE_POSITION);
                        break;
                }
                break;
            case -1: //Depot
                switch (Intake.intakeState) {
                    case 2:
//                        Robot.wrist.setAbsoluteAngle(DriverStation.getInstance().isAutonomous() ? RobotMap.WRIST_CARGO_SHIP_INTAKE_ANGLE : RobotMap.WRIST_EXTENDED_ANGLE);
//                        Robot.elevator.setAbsoluteHeight(DriverStation.getInstance().isAutonomous() ? RobotMap.ELEVATOR_CARGO_AUTO_INTAKE_POSITION : RobotMap.ELEVATOR_CARGO_INTAKE_DEPOT_POSITION);
	                    Robot.wrist.setAbsoluteAngle(WRIST_SETPOINTS.EXTENDED);
	                    Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.CARGO_INTAKE_DEPOT);
//                        Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_INTAKE_SPEED);
                        break;
                    case 1:
                    case 0:
                    default:
                        break;
                }
                break;
            case 0: // Elevator, Wrist Home Position
            default:
                Robot.wrist.setAbsoluteAngle(Intake.intakeState == 2 ? WRIST_SETPOINTS.CARGO_RETRACTED : WRIST_SETPOINTS.RETRACTED);
                Robot.elevator.setAbsoluteHeight(ELEVATOR_SETPOINTS.HOME);
                Robot.intake.setCargoIntakeOutput(Robot.intake.bannerIR.get() ? INTAKE_SPEED.CARGO_HOLD : 0);
                Robot.harpoon.setHarpoonExtend(false);
                break;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
