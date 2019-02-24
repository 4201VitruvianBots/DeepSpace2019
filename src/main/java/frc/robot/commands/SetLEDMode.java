/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetLEDMode extends Command {
  int pinNumber;                    //creates an integer pinNumber to match the parameter one
  public SetLEDMode(int pinNumber) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.ledOutput);
    this.pinNumber = pinNumber;   //sets the variable pinNumber to the value of parameter pinNumber
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean prePinState = Robot.ledOutput.getDIOState(pinNumber); //saves the pin state so we know if it was off or on
    Robot.ledOutput.setPinOutput(false,0);  //turns everything off so no two pins are active at once
    Robot.ledOutput.setPinOutput(false,1);
    Robot.ledOutput.setPinOutput(false,2);
    Robot.ledOutput.setPinOutput(false,3);
    if(!prePinState){      //checks if the pin was
      Robot.ledOutput.setPinOutput(true,pinNumber);  //if it is turn it  on
    }
    isFinished();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.ledOutput.setPinOutput(false,0);  //should turn everything off if another LED command is called
    Robot.ledOutput.setPinOutput(false,1);
    Robot.ledOutput.setPinOutput(false,2);
    Robot.ledOutput.setPinOutput(false,3);
    end();
  }
}
