package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ZeroElevatorEncoder extends InstantCommand {
    public ZeroElevatorEncoder() {
        requires(Robot.elevator);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.elevator.setEncoderZeroOffset(0);
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
