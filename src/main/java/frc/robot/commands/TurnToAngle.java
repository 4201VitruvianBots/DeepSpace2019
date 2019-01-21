/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class TurnToAngle extends PIDCommand {
    static double kP = 0.04;   //0.1
    static double kI = 0;
    static double kD = 0;  //10
    static double kF = 0;  //1023.0 / 72000.0;
    static double period = 0.02;

    double targetAngle;
    double output;
    public TurnToAngle(double targetAngle) {
        super(kP, kI, kD, period);

        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        this.targetAngle = targetAngle;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.setDriveMotorsState(false);
        getPIDController().setF(kF);
        getPIDController().setContinuous(true);
        getPIDController().setAbsoluteTolerance(1.5);
        getPIDController().setOutputRange(-1, 1); // +/- 0.8

        double currentAngle = Robot.driveTrain.navX.getAngle();
        setSetpoint(currentAngle + targetAngle);
        getPIDController().enable();
    }

    @Override
    protected double returnPIDInput() {
        return Robot.driveTrain.navX.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        this.output = output;
    }


    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return getPIDController().onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        getPIDController().disable();
        Robot.driveTrain.setMotorTankDrive(0, 0);
        Robot.driveTrain.setDriveMotorsState(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }


}
