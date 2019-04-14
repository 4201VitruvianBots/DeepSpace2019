/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.KillAll;

/**
 * An example command.  You can replace me with your own command.
 */
public class EnableClimbSequence extends CommandGroup {
    public EnableClimbSequence() {
        addSequential(new SetClimbMode(1));
        addSequential(new SetWristElevatorClimbPositions(true));
        addSequential(new SetClimbPistons(true));
        addSequential(new SetCompressorState(false));
        addSequential(new KillAll());
        addSequential(new SetLimelightLEDMode(1));
        addSequential(new SetClimbLEDIndicators(true));
    }
}
