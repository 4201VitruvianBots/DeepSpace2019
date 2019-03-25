/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

/**
 * An example command.  You can replace me with your own command.
 */
public class UpdateWristSetpoint extends Command {

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
        double joystickOutput = Robot.m_oi.getXBoxRightY();

        if (Wrist.controlMode == 1) {
            if(Math.abs(joystickOutput) > 0.05) {
                double setpoint = joystickOutput * 10;

                // TODO: Change this logic to use limit switches when they are fixed
                if(setpoint <= 0 && Robot.wrist.getAngle() < 0.1 || setpoint >= 120  && Robot.wrist.getAngle() > 119.9)
                    Robot.m_oi.enableXBoxRumbleTimed(0.2);

                Robot.wrist.setIncrementedPosition(setpoint);
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
