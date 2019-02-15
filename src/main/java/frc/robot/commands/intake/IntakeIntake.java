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
public class IntakeIntake extends Command {
    Timer pause = new Timer();
    public IntakeIntake() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.wrist);
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                // TODO: Set wrist ground
                break;
            case 0:
            default:
                // TODO: Set wrist retract
                break;
        }
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
                Robot.intake.setCargoIntakeOutput(-0.8);
                break;
            case 1:
                Robot.intake.setHatchGroundIntakeOutput(-0.8);
                break;
            case 0:
            default:
                Robot.intake.setHarpoonExtend(true);
                Robot.intake.setHarpoonSecure(false);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        if(Intake.intakeState == 2)
            return Robot.intake.bannerIR.get();
        else
            return false || !Robot.m_oi.leftButtons[0].get();
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        pause.reset();
        switch (Intake.intakeState) {
            case 2:
                if(Robot.intake.bannerIR.get())
                   Robot.intake.setCargoIntakeOutput(-0.2);
                else
                    Robot.intake.setCargoIntakeOutput(0);
                break;
            case 1:
                Robot.intake.setCargoIntakeOutput(0);
                pause.start();
                while (pause.get() < 0.15) {

                }
                //TODO: Retract wrist to home.
                pause.stop();
                break;
            case 0:
            default:
                Robot.intake.setHarpoonSecure(true);
                pause.start();
                while(pause.get() < 0.15) {

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
