/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.harpoon;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetHarpoonExtend extends InstantCommand {
    boolean extend;

    public SetHarpoonExtend(boolean extend) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.harpoon);

        this.extend = extend;
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
                Robot.harpoon.setHarpoonExtend(extend);
                break;
        }
    }

    @Override
    protected void end() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
                break;
            case 0:
            default:
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
