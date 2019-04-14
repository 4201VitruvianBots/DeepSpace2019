package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class ToggleEnableBannerSensor extends InstantCommand {
    public ToggleEnableBannerSensor() {
        requires(Robot.intake);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Intake.enableBannerSensor = !Intake.enableBannerSensor;
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
