/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

/**
 * An example command.  You can replace me with your own command.
 */
public class ToggleWristState extends InstantCommand {
    public ToggleWristState() {
        requires(Robot.wrist);
        setTimeout(0.2);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Scheduler.getInstance().removeAll();
        if(Wrist.controlMode == 1)
            Wrist.controlMode = 0;
        else {
            Robot.wrist.setEncoderPosition(Wrist.upperLimitEncoderCounts);
            Wrist.controlMode = 1;
        }
        Robot.m_oi.enableXBoxRumbleTimed(0.2);
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
