/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.vitruvianlib.util.DummyPIDOutput;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoldToAlignWithTargetNotifier extends Command {
    double kP = 0.04;   //0.1
    double kI = 0;
    double kD = 0;  //10
    double kF = 0;  //1023.0 / 72000.0;
    DummyPIDOutput turnOutput = new DummyPIDOutput();
    PIDController turnPID = new PIDController(kP, kI, kD, kF, Robot.driveTrain.navX, turnOutput);
    double lastLimelightAngle = 0;

    Notifier periodicRunnable;
    boolean isFinished = false;

    public HoldToAlignWithTargetNotifier() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.setDriveMotorsState(false);

        turnPID.setOutputRange(-0.5, 0.5);
        turnPID.setSetpoint(Robot.driveTrain.navX.getAngle() + Robot.vision.getTargetX());
        turnPID.enable();
        periodicRunnable = new Notifier(new PeriodicRunnable());
        periodicRunnable.startPeriodic(0.02);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished(){
        return isFinished;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        turnPID.disable();
        periodicRunnable.stop();
        Robot.driveTrain.leftAdjustment = 0;
        Robot.driveTrain.rightAdjustment = 0;
        Robot.driveTrain.setDriveMotorsState(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }


    public class PeriodicRunnable implements Runnable {

        @Override
        public void run() {
            double throttle = Robot.m_oi.getLeftJoystickY();
            double turn;

            if (Robot.vision.isValidTarget()) {
                /*

                double limelightAngle = Robot.vision.getTargetX();
                lastLimelightAngle = limelightAngle;
                if(limelightAngle != lastLimelightAngle) {
                    double currentNavXAngle = Robot.driveTrain.navX.getAngle();
                    double setpoint = currentNavXAngle + lastLimelightAngle;
                    turnPID.setSetpoint(setpoint);
                }
                */
                turnPID.enable();
                turnPID.setSetpoint(0);
                turn = -turnOutput.getOutput();
            } else {
                turnPID.disable();
                turn = Robot.m_oi.getRightJoystickX();
            }

            if (Robot.driveTrain.getTalonControlMode() == ControlMode.Velocity)
                Robot.driveTrain.setArcadeDriveVelocity(throttle, turn);
            else
                Robot.driveTrain.setMotorArcadeDrive(throttle, turn);

            if(Robot.vision.IsTargetGood())
                isFinished = true;
        }
    }
}
