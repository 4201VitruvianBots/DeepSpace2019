/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

/**
 * An example command.  You can replace me with your own command.
 */
public class UpdateWristSetpoint extends Command {
	Timer stopwatch = new Timer();
	boolean mutex = false;
	
    public UpdateWristSetpoint() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.wrist);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double joystickInput = Math.abs(Robot.m_oi.getXBoxRightY()) > 0.05 ? Robot.m_oi.getXBoxRightY() : 0;
        double joystickOutput = 0.5 * (joystickInput + Math.pow(joystickInput, 3));

        if (Wrist.controlMode == 1) {
            double setpoint = joystickOutput * 10;

            // TODO: Change this logic to use limit switches when they are fixed
            if(setpoint <= 0 && Robot.wrist.getAngle() < 0.1 || setpoint >= 120  && Robot.wrist.getAngle() > 119.9)
                Robot.m_oi.enableXBoxRumbleTimed(0.2);

            Robot.wrist.setIncrementedPosition(setpoint);

            if(Robot.wrist.getOutputCurrent() > 15) {
        		if(!mutex) {
        			mutex = true;
        			stopwatch.reset();
        			stopwatch.start();
        		}
        		if(stopwatch.get() > 1) {
                	mutex = false;
                	stopwatch.stop();

                    Robot.wrist.setAbsolutePosition(Robot.wrist.getAngle());
        		}
            } else if(mutex) {
            	mutex = false;
            	stopwatch.stop();
            }
        } else {
            // TODO: Uncomment once limit switches are implemented
            /*if(Robot.wrist.getLimitSwitchState(0) || Robot.wrist.getLimitSwitchState(1)) {
                joystickOutput = 0;
                Robot.m_oi.setXBoxRumble(0.8);
            } else
                Robot.m_oi.setXBoxRumble(0);*/
            Robot.wrist.setDirectOutput (joystickOutput);
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
    }
}
