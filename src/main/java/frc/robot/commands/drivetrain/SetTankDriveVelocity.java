/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetTankDriveVelocity extends Command {
    public SetTankDriveVelocity() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double leftInput = Robot.m_oi.getLeftJoystickY();
        double rightInput = Robot.m_oi.getRightJoystickY();

        double leftOutput = (Math.abs(leftInput) > 0.05) ? leftInput : 0;
        double rightOutput = (Math.abs(rightInput) > 0.05) ? rightInput : 0;

        Robot.driveTrain.setMotorVelocityOutput(leftOutput, rightOutput);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setMotorVelocityOutput(0, 0);
        //Robot.driveTrain.setMotorGains(0, 0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
