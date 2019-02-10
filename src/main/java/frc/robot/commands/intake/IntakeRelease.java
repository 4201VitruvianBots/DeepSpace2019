/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeRelease extends Command {
    Timer pause = new Timer();

    public IntakeRelease() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.wrist);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
                Robot.intake.setCargoIntakeOutput(0.8);
                break;
            case 1:
            case 0:
            default:
                Robot.intake.setHarpoonExtend(true);
                Robot.intake.setHarpoonSecure(true);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        pause.reset();
        switch (Intake.intakeState) {
            case 2:
                Robot.intake.setCargoIntakeOutput(0);
                break;
            case 1:
            case 0:
            default:
                Robot.intake.setHarpoonSecure(false);
                pause.start();
                while(pause.get() < 0.35) {

                }
                Robot.intake.setHarpoonExtend(false);
                pause.stop();
                break;
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
