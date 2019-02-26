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
    static double kP = 0.003;   //0.1
    static double kI = 0;
    static double kD = 0;  //10
    static double kF = 0;  //1023.0 / 72000.0;
    static double period = 0.02;
    double lastLimelightAngle = 0;
    double throttle, turn;
    //Notifier periodicRunnable;
    boolean isFinished = false;

    public HoldToAlignWithTarget() {
        super(kP, kI, kD, period);
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.setDriveMotorsState(false);
        getPIDController().setF(kF);
        //getPIDController().get
        //getPIDController().setContinuous(true);
        getPIDController().setAbsoluteTolerance(1);
        getPIDController().setOutputRange(-0.5, 0.5);

        Robot.vision.setPipeline(1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

        if (Robot.vision.isValidTarget()) {
            double limelightAngle = Robot.vision.getTargetX();
            lastLimelightAngle = limelightAngle;
            getPIDController().enable();
            if(limelightAngle != lastLimelightAngle) {
                double currentNavXAngle = Robot.driveTrain.navX.getAngle();
                double setpoint = currentNavXAngle - lastLimelightAngle;
                getPIDController().setSetpoint(setpoint);
            }
        } else {
            getPIDController().disable();
            turn = Robot.m_oi.getRightJoystickX();
        }
    }

    @Override
    protected double returnPIDInput() {
        return Robot.driveTrain.navX.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        double adjustment = -output;
        throttle = Robot.m_oi.getLeftJoystickY();
        turn = Robot.m_oi.getRightJoystickX();

        double leftOutput = throttle - turn - adjustment;
        double rightOutput = throttle + turn + adjustment;

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
        Robot.vision.setPipeline(1);
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
