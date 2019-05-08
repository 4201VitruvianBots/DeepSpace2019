/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDOutput;
import edu.wpi.first.wpilibj.command.Command;

public class UpdateLEDState extends Command {

    public UpdateLEDState() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.ledOutput);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	Robot.ledOutput.updateLEDState();
    }                                                                        

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.ledOutput.setPinOutput(false, 0);  //should turn everything off if another LED command is used
        Robot.ledOutput.setPinOutput(false, 1);
        Robot.ledOutput.setPinOutput(false, 2);
        Robot.ledOutput.setPinOutput(false, 3);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
