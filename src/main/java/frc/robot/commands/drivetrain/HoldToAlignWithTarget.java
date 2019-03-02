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
public class HoldToAlignWithTarget extends PIDCommand {
    static double kP = 0.015;   //0.1
    static double kI = 0;
    static double kD = 0;  //10
    static double kF = 0;  //1023.0 / 72000.0;
    static double period = 0.02;


    boolean isFinished = false;

    double lastTx = 0;

    public HoldToAlignWithTarget() {
        super(kP, kI, kD, period);
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        lastTx = 0;
        Robot.driveTrain.setDriveMotorsState(false);
        getPIDController().setF(kF);
        getPIDController().setInputRange(-180.0f, 180.0f);
        getPIDController().setContinuous(true);
        getPIDController().setAbsoluteTolerance(1);
        getPIDController().setOutputRange(-1, 1);

        //Robot.vision.setPipeline(1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (Robot.vision.isValidTarget()) {
            double targetRatio = 0;
            try {
                targetRatio = Robot.vision.getTShort() / Robot.vision.getTLong();
            } catch (Exception e) {

            }
            if(targetRatio > .45 || targetRatio < 0.15)
                lastTx = lastTx;
            else {
                lastTx = Robot.vision.getTargetX();
            }

            getPIDController().setSetpoint(lastTx);
            getPIDController().enable();
        } else
            getPIDController().disable();
    }

    @Override
    protected double returnPIDInput() { return Robot.driveTrain.navX.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        double leftOutput = (Robot.m_oi.getLeftJoystickY() - Robot.m_oi.getRightJoystickX()) - output;
        double rightOutput = (Robot.m_oi.getLeftJoystickY() + Robot.m_oi.getRightJoystickX()) + output;

        if (Robot.driveTrain.getTalonControlMode() == ControlMode.Velocity)
            Robot.driveTrain.setMotorVelocityOutput(leftOutput, rightOutput);
        else
            Robot.driveTrain.setMotorPercentOutput(leftOutput, rightOutput);
    }

    @Override
    protected boolean isFinished(){
        return false; //getPIDController().onTarget();
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        //Robot.vision.setPipeline(1);
        getPIDController().disable();
        Robot.driveTrain.setDriveMotorsState(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
