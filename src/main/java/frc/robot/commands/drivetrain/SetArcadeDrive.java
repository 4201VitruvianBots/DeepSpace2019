/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetArcadeDrive extends Command {

    public static boolean override = false;

    public SetArcadeDrive() {
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
        if(Robot.m_oi.usingExpensiveJoysticks) {
            Robot.driveTrain.setMotorArcadeDrive(Robot.m_oi.getLeftJoystickY(), 0.375 * Robot.m_oi.getRightJoystickX());
        } else {
            Robot.driveTrain.setMotorArcadeDrive(Robot.m_oi.getLeftJoystickY(),0.8 * Robot.m_oi.getRightJoystickX());
        }
/*        double joystickY = (Math.abs(Robot.m_oi.getLeftJoystickY()) > 0.0) ? Robot.m_oi.getLeftJoystickY() : 0;
        double joystickX = (Math.abs(Robot.m_oi.getRightJoystickX()) > 0.0) ? Robot.m_oi.getRightJoystickX() : 0;

//        double throttle = 0.5 * (joystickY + Math.pow(joystickY, 3));
//        throttle = throttle < 0 ? Math.max( -0.7, throttle) : throttle;
//        double turn = 0.25 *(joystickX + Math.pow(joystickX, 3));
        double throttle = joystickY;
        throttle = throttle < 0 ? Math.max(-0.7, throttle) : throttle;
        if(Robot.elevator.controlMode == 1)
            throttle = Robot.elevator.getHeight() > 30 ? Math.min(Math.max(throttle, -0.4), 0.5): throttle;
        double turn = (Robot.driveTrain.getDriveShifterStatus() ? 0.5 : 0.35) * joystickX;
        turn = (Robot.driveTrain.getDriveShifterStatus() ? Math.max(turn, 0.5) : Math.max(turn, 0.35));

        if (Robot.climber.climbMode == 1) {
            double operatorThrottle = Math.abs(Robot.m_oi.getXBoxRightY()) > 0.05 ? Robot.m_oi.getXBoxRightY() : 0;
            Robot.driveTrain.setClimbMotorPercentOutput(Math.min(throttle + operatorThrottle, 0.5));
//            Robot.driveTrain.setClimbMotorCurrentOutput(30 * Math.min(throttle + operatorThrottle, 0.5));
            throttle = Math.max(Math.min(throttle, 0.25), -0.25);
            turn = Math.max(Math.min(turn, 0.4), -0.4);
            Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
        } else {
            if (Robot.elevator.controlMode == 1)
                throttle = Robot.elevator.getHeight() > 30 ? Math.min(Math.max(throttle, -0.4), 0.5) : throttle;
            Robot.driveTrain.setMotorArcadeDrive(throttle, turn);
        }
        */
    }
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setMotorTankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
