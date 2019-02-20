package frc.robot.commands.test;

import edu.wpi.first.wpilibj.InterruptableSensorBase;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

public class ZeroWristEncoder extends InstantCommand {
    public ZeroWristEncoder() {
        requires(Robot.wrist);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.wrist.setEncoderPosition(Wrist.upperLimitEncoderCounts);
        int calibrationValue = -Robot.wrist.getPosition();
//        Robot.wrist.setEncoderPosition(calibrationValue);
        Robot.controls.writeIniFile("Wrist", "Encoder_Calibration", String.valueOf(calibrationValue));
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
