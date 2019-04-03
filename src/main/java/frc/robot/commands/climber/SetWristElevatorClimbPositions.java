/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.wrist.SetWristSetpoint;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetWristElevatorClimbPositions extends CommandGroup {
    public SetWristElevatorClimbPositions() {
        addParallel(new SetWristSetpoint(RobotMap.WRIST_EXTENDED_ANGLE));
        addParallel(new SetElevatorSetpoint(RobotMap.ELEVATOR_CLIMB_POSITION));
    }
}
