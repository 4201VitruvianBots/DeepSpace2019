/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * An example command.  You can replace me with your own command.
 */
public class ClimbModeSequence extends CommandGroup {
    public ClimbModeSequence() {
        addSequential(new SetClimbMode());
        addSequential(new SetWristElevatorClimbPositions());
        addSequential(new SetGreyClimberPistons());
//        addSequential(new DisableLimelightLEDs());
//        addSequential(new SetClimbLEDIndicators());
//        addSequential(new KillAll());
    }
}
