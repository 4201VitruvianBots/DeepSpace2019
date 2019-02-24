/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDOutput;

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
    switch (LEDOutput.state) {
      case 3:      //LEDCh3 on, should be green.
        Robot.ledOutput.setPinOutput(true,3);
        Robot.ledOutput.setPinOutput(false,2);
        Robot.ledOutput.setPinOutput(false,1);
        Robot.ledOutput.setPinOutput(false,0);
        break;
      case 2:     //LEDCh2 on, should be magenta
        Robot.ledOutput.setPinOutput(false,3);
        Robot.ledOutput.setPinOutput(true,2);
        Robot.ledOutput.setPinOutput(false,1);
        Robot.ledOutput.setPinOutput(false,0);
        break;
      case 1:     //LEDCh1 on, should be red
        Robot.ledOutput.setPinOutput(false,3);
        Robot.ledOutput.setPinOutput(false,2);
        Robot.ledOutput.setPinOutput(true,1);
        Robot.ledOutput.setPinOutput(false,0);
        break;
      case 0:     //LEDCh0 on, should be green
        Robot.ledOutput.setPinOutput(false,3);
        Robot.ledOutput.setPinOutput(false,2);
        Robot.ledOutput.setPinOutput(false,1);
        Robot.ledOutput.setPinOutput(true,0);
        break;
      case -1:    //This should only really happen when the robot is disabled
      default:
        Robot.ledOutput.setPinOutput(false,3);
        Robot.ledOutput.setPinOutput(false,2);
        Robot.ledOutput.setPinOutput(false,1);
        Robot.ledOutput.setPinOutput(false,0);
        break;
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
