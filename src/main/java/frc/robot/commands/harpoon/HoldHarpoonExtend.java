/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoldHarpoonExtend extends Command {
    public HoldHarpoonExtend() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.harpoon);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                break;
            case 0:
            default:
                break;
        }
    }

    @Override
    protected void execute() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                break;
            case 0:
            default:
                Robot.harpoon.setHarpoonExtend(true);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                break;
            case 0:
            default:
                Robot.harpoon.setHarpoonExtend(false);
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
