/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetElevatorSetpointWait extends Command {
    double setpoint;
    public SetElevatorSetpointWait(double position) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elevator);
        this.setpoint = position;
        setTimeout(1);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if(Elevator.controlMode == 1)
            Robot.elevator.setAbsoluteHeight(setpoint);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.elevator.getHeight() - setpoint) < 0.2 || isTimedOut();
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
