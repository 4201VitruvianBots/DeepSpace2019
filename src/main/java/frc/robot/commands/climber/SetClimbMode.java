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
public class SetClimbMode extends CommandGroup {
    public SetClimbMode() {
        addSequential(new SetWristElevatorClimbPositions());
//        addSequential(new ToggleGreyClimber());
        addSequential(new KillAll());
    }
}
