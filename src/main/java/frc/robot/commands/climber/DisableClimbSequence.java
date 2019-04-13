/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.ReviveAll;
import frc.robot.commands.elevator.SetElevatorLimitBreak;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.wrist.SetWristSetpoint;

/**
 * An example command.  You can replace me with your own command.
 */
public class DisableClimbSequence extends CommandGroup {
    public DisableClimbSequence() {
        addSequential(new SetClimbMode(1));
        addSequential(new SetClimbLEDIndicators(false));
        addSequential(new SetElevatorLimitBreak(false));
        addSequential(new ReviveAll());
        addSequential(new SetElevatorSetpoint(RobotMap.ELEVATOR_CLIMB_POSITION));
        addSequential(new SetClimbPistons(false));
        addSequential(new SetWristSetpoint(RobotMap.WRIST_RETRACTED_ANGLE));
        addSequential(new SetClimbLEDIndicators(false));
    }
}
