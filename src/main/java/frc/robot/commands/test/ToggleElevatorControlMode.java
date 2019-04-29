package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.vitruvianlib.driverstation.Shuffleboard;

public class ToggleElevatorControlMode extends InstantCommand {
    public ToggleElevatorControlMode() {
        requires(Robot.elevator);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.elevator.controlMode = Robot.elevator.controlMode == 1 ? 0 : 1;
    }
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
