/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetDriveShifters extends InstantCommand {
    public SetDriveShifters() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if (Robot.driveTrain.getDriveShifterStatus())
            Robot.driveTrain.setDriveShifterStatus(false);
        else
            Robot.driveTrain.setDriveShifterStatus(true);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
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
