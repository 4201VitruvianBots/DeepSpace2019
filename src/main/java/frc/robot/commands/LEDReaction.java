/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

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
    Robot.ledOutput.setPinOutput(false,2);
    Robot.ledOutput.setPinOutput(false,3);
    if(Robot.ledOutput.getBannerState()){                //check if the limit switch is open
      Robot.ledOutput.setPinOutput(true,1);  // if it is turn pin 1 on
      Robot.ledOutput.setPinOutput(false, 0);
    }
    else{
      Robot.ledOutput.setPinOutput(true, 0);  //if it's closed turn pin 0 on
      Robot.ledOutput.setPinOutput(false, 1);
    }
  }

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
