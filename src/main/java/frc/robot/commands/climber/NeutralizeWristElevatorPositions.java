/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.elevator.SetElevatorSetpointWait;
import frc.robot.commands.wrist.SetWristSetpoint;
import frc.robot.commands.wrist.SetWristSetpointWait;

/**
 * An example command.  You can replace me with your own command.
 */
public class NeutralizeWristElevatorPositions extends CommandGroup {
    public NeutralizeWristElevatorPositions() {
        addParallel(new SetWristSetpoint(Robot.wrist.getAngle()));
        addParallel(new SetElevatorSetpoint(Robot.elevator.getHeight()));
    }
}