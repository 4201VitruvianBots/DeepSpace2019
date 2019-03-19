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

public class FollowVisionTargetv2 extends PIDCommand {
    static double kP = 0.015; //Proportion for turning
    static double kI = 0; //Proportion for turning
    static double kD = 0; //Proportion for turning
    double tta = 0.85; //Target TA val

    static double leftAdjustment = 1;
    static double rightAdjustment = 1;
    int rescanCounter = 0;

    public FollowVisionTargetv2() {
        super(kP, kI, kD);
        // Use requires() here to declare subsystem dependencies
        // requires(Robot.m_subsystem);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        //Robot.vision.setPipeline(1);
        Robot.driveTrain.setDriveMotorsState(true);

        this.getPIDController().setAbsoluteTolerance(1);
        this.getPIDController().setOutputRange(-1, 1);
    }

    @Override
    protected double returnPIDInput() {
        return Robot.vision.getTargetX();
    }

    @Override
    protected void usePIDOutput(double output) {
        Robot.driveTrain.setMotorPercentOutput(leftAdjustment * (Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX()) - output, rightAdjustment * (Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX()) + output);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.vision.isValidTarget()) {
            if (rescanCounter % 5 == 0) {
                Robot.vision.setPipeline(4);
                double leftArea = Robot.vision.getTargetArea();

                Robot.vision.setPipeline(5);
                double rightArea = Robot.vision.getTargetArea();

                double diff = leftArea -  rightArea;

                if(diff > 0.5) {
                    leftAdjustment = 0.6;
                } else if(diff < -0.5) {
                    rightAdjustment = 0.6;
                } else {
                    leftAdjustment = 1;
                    rightAdjustment = 1;
                }
            }
            this.getPIDController().enable();
            rescanCounter++;
        } else {
            this.getPIDController().disable();
            rescanCounter = 0;
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
        //Robot.vision.setPipeline(1);
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