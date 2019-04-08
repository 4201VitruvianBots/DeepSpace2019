package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class IntakeHatch extends Command {
    Timer stopwatch = new Timer();

    public IntakeHatch() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Intake.intakeState == 0)
            Robot.intake.setHatchIntakeOutput(RobotMap.HATCH_INTAKE_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(Intake.intakeState == 0) {
            stopwatch.reset();
            stopwatch.start();
            while (stopwatch.get() < 0.25) {

            }
            stopwatch.stop();
            Robot.intake.setHatchIntakeOutput(0);
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
