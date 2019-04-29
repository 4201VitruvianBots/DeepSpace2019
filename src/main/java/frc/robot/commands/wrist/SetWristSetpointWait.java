/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetWristSetpointWait extends Command {
    double setpoint;
    public SetWristSetpointWait(double angle) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.wrist);
        this.setpoint = angle;
        setTimeout(1.5);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if(Robot.wrist.controlMode == 1)
            Robot.wrist.setAbsoluteAngle(setpoint);
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(Robot.wrist.getAngle() - setpoint) < 0.5 || isTimedOut();
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
