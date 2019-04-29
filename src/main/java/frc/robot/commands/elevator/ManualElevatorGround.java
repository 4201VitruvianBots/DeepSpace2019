package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ManualElevatorGround extends Command {
    public ManualElevatorGround() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elevator);
        setTimeout(5);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.elevator.setOpenLoopOutput(-0.4);
    }


    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return isTimedOut() || Robot.elevator.getLimitSwitchState(0);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(Robot.elevator.getLimitSwitchState(0)) {
            Robot.elevator.controlMode = 1;
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
