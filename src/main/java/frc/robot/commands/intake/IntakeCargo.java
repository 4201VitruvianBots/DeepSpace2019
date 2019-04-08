package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends Command {
    Timer stopwatch = new Timer();

    public IntakeCargo() {
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
        if(Intake.intakeState == 2)
            Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_INTAKE_SPEED);
    }

    @Override
    protected boolean isFinished() {
        if(Intake.intakeState == 2)
            return Robot.intake.bannerIR.get();
        else
            return true;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        if(Intake.intakeState == 2) {
            if (Robot.intake.bannerIR.get()) {
                stopwatch.reset();
                stopwatch.start();
                while (stopwatch.get() < 0.5) {

                }
                stopwatch.stop();
                Robot.wrist.setAbsolutePosition(RobotMap.WRIST_RETRACTED_CARGO_ANGLE);
//                    Timer.delay(0.1);
                Robot.intake.setCargoIntakeOutput(RobotMap.CARGO_HOLD_SPEED);
            } else {
                Robot.intake.setCargoIntakeOutput(0);
            }
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
