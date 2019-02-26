/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */

public class FollowVisionTarget extends PIDCommand {
    static double kP = 0.015; //Proportion for turning
    static double kI = 0; //Proportion for turning
    static double kD = 0; //Proportion for turning
    double tta = 0.85; //Target TA val

    static double lastTx = 0;
    public FollowVisionTarget() {
        super(kP, kI, kD);
        // Use requires() here to declare subsystem dependencies
        // requires(Robot.m_subsystem);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        lastTx = 0;
        Robot.vision.setPipeline(1);
        Robot.driveTrain.setDriveMotorsState(false);
        this.getPIDController().setAbsoluteTolerance(1);
        this.getPIDController().setOutputRange(-1, 1);
    }

    @Override
    protected double returnPIDInput() {
        double targetRatio = Robot.vision.getTShort() / Robot.vision.getTLong();

        if(targetRatio > .45 || targetRatio < 0.15)
            lastTx = lastTx;
        else {
            lastTx = Robot.vision.getTargetX();
        }
            return lastTx;
    }

    @Override
    protected void usePIDOutput(double output) {
        double leftOutput = (Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX()) - output;
        double rightOutput = (Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX()) + output;

        if(lastTx > 15)
            leftOutput = leftOutput * 0.8;
        else if(lastTx < -15)
            rightOutput = rightOutput * 0.8;

        Robot.driveTrain.setMotorPercentOutput(leftOutput, rightOutput);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.vision.isValidTarget()) {
            this.getPIDController().enable();
            //double correction = Robot.limelight.getTargetX() * kP;
            //Robot.driveTrain.setDriveOutput((Robot.m_oi.getLeftY() - Robot.m_oi.getRightX()) + correction, (Robot.m_oi.getLeftY() + Robot.m_oi.getRightX()) - correction);
        } else
            this.getPIDController().disable();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setDriveMotorsState(true);
        //Robot.driveTrain.setDriveOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}