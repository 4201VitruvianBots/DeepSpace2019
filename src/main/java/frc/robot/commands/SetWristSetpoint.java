/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetWristSetpoint extends InstantCommand {
    double setpoint;
    public SetWristSetpoint(double angle) {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.elevator);
        this.setpoint = angle;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
