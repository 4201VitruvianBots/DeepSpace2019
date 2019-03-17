/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.LEDOutput;
import edu.wpi.first.wpilibj.command.Command;

public class LEDReaction extends Command {

    public LEDReaction() {
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
        if(Robot.ledOutput.getShifterState){    //if we're in low(er) gear
            Robot.ledOutput.setPinOutput(true,0);   //first bit on
        }
        else{   //if we're in "high" gear
            Robot.ledOutput.setPinOutput(false,0);  //first bit on
        }
       Robot.ledOutput.setPinOutput((LEDOutput.LEDColour % 8 > 3), 1);  //checks what each digit of the state # is
       Robot.ledOutput.setPinOutput((LEDOutput.LEDColour % 4 > 1), 2);  //in binary, with pin 1 as a 4s place, 2 as
       Robot.ledOutput.setPinOutput((LEDOutput.LEDColour % 2 > 0), 3);  //2s, and 3 as 1s. Pin on for 1 & off for 0.
    }                                                                        //If # is > 7, the binary # overflows.

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.ledOutput.setPinOutput(false,0);  //should turn everything off if another LED command is used
        Robot.ledOutput.setPinOutput(false,1);
        Robot.ledOutput.setPinOutput(false,2);
        Robot.ledOutput.setPinOutput(false,3);
        end();
    }
}
