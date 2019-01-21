/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoldToFollowTarget extends Command {
    double kP = 0.04; //Proportion for turning
    double kPB = 1.4; //Proportion for moving
    double ds = 0.5; //Default speed multiplier
    double tta = 0.85; //Target TA val

    public HoldToFollowTarget() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.setDriveMotorsState(false);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.vision.isValidTarget()) {
            double correction = Robot.vision.getTargetX() * kP;
            double paddingCorrection = ds*((tta - Robot.vision.getTargetX()) * kPB);
            Robot.driveTrain.setMotorVelocityOutput(-paddingCorrection + correction, -paddingCorrection - correction);
        }
    }

    @Override
    protected boolean isFinished() {
        if(Robot.driveTrain.getLeftEncoderVelocity() <= 0 && Robot.driveTrain.getLeftEncoderVelocity() <= 0){
            return true;
        }
        else {
            return false;
        }
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setDriveMotorsState(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
