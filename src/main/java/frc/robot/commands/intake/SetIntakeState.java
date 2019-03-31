/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetIntakeState extends InstantCommand {
    int state;
    public SetIntakeState(int state) {
        requires(Robot.intake);
        requires(Robot.intakeExtend);
        this.state = state;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if(state == 2) {
            //Robot.intake.setHarpoonSecure(false);
            Robot.intake.setHatchIntakeOutput(0);
            Robot.intakeExtend.setHarpoonExtend(false);
        }

        if(state != 2)
            Robot.intake.setCargoIntakeOutput(0);

        Intake.intakeState = state;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
