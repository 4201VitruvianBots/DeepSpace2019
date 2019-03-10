/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.LEDOutput;

/**
 * An example command.  You can replace me with your own command.
 */
public class CycleLEDStates extends InstantCommand {

    private static int stateCount = 0;   //how many times have we cycled through this
    public CycleLEDStates() {
        requires(Robot.ledOutput);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        LEDOutput.state = (stateCount++ % 5) - 1;   //LED State will be -1 - 4 depending on how many times we've done this
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
