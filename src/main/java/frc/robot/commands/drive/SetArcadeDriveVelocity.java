/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetArcadeDriveVelocity extends Command {
    public SetArcadeDriveVelocity() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        //Robot.driveTrain.setMotorGains(0.25, 0.001, 20, 1023.0/72000.0);
        Robot.driveTrain.setMotorGains(0.25, 0, 10, 1023.0 / 72000.0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double joystickY = Robot.m_oi.getLeftJoystickY();
        double joystickX = Robot.m_oi.getRightJoystickX() * 0.125;

        // Deadzone code, not really used ATM
        double throttle = (Math.abs(joystickY) > 0.0) ? joystickY : 0;
        double turn = (Math.abs(joystickX) > 0.0) ? joystickX : 0;

        Robot.driveTrain.setArcadeDriveVelocity(throttle, turn);
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
        Robot.driveTrain.setMotorGains(0, 0, 0, 0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
