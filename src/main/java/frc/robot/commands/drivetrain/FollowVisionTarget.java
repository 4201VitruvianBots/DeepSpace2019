/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */

public class FollowVisionTarget extends PIDCommand {
    static double kP = 0.02; //Proportion for turning
    static double kI = 0; //Proportion for turning
    static double kD = 0.00015;
    //Proportion for turning
    static double kF = 0.2; //Proportion for turning
    double tta = 0.85; //Target TA val

    double lastTx = 0;
    double lastRatio = 0;
    double idealRatio = 5.825 / 14.5;

    double[] targetsArray;

    boolean targetLocked = false;

    public FollowVisionTarget() {
        super(kP, kI, kD);
        this.getPIDController().setF(kF);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        lastTx = 0;
        targetLocked = false;
        //Robot.driveTrain.setDriveMotorsState(false);
        this.getPIDController().setAbsoluteTolerance(1);
        this.getPIDController().setOutputRange(-1, 1);

        Robot.vision.setPipeline(0);
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

    @Override
    protected double returnPIDInput() {
//        double targetRatio = Robot.vision.getTShort() / Robot.vision.getTLong();
        double currentTx = -Robot.vision.getTargetX();

//        if(Robot.vision.isValidTarget())
//            lastTx = currentTx;

        if(Math.abs(currentTx) < 3)
            targetLocked = true;

        if(targetLocked)
            getPIDController().setD(0);

//        if(targetRatio < 0.45 && targetRatio > 0.15)
//            lastTx = currentTx;
//            lastRatio = targetRatio;
//        }
//        lastTx = Robot.vision.getTargetSkew() > 10 ? lastTx : currentTx;
//        lastTx = (targetRatio > .45 || targetRatio < 0.15) ? lastTx : currentTx;
//        lastTx = (Math.abs(currentTx - lastTx) > 10) ? lastTx : currentTx;
//        lastTx = (Math.abs(currentTx - lastTx) < 10) && Math.abs(lastTx) < Math.abs(currentTx) ? lastTx : currentTx;

        return currentTx;
    }

    @Override
    protected void usePIDOutput(double output) {
//        double leftOutput = (Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX()) + output;
//        double rightOutput = (Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX()) - output;

//        leftOutput = lastTx > 15 ? leftOutput * 0.8 : leftOutput;
//        rightOutput = lastTx < -15 ? rightOutput * 0.8 : rightOutput;
        double leftOutput = 0;
        double rightOutput = 0;
        if(Robot.vision.isValidTarget()) {
            leftOutput = Robot.m_oi.getLeftJoystickY() + output;
            rightOutput = Robot.m_oi.getLeftJoystickY() - output;
        } else {
            leftOutput = Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX();
            rightOutput = Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX();
        }

        if (Robot.driveTrain.getTalonControlMode() == ControlMode.Velocity)
            Robot.driveTrain.setMotorVelocityOutput(leftOutput, rightOutput);
        else
            Robot.driveTrain.setMotorPercentOutput(leftOutput, rightOutput);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        getPIDController().disable();
        //Robot.driveTrain.setDriveMotorsState(true);
        //Robot.driveTrain.setDriveOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}