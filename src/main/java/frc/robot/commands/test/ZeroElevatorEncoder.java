package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ZeroElevatorEncoder extends InstantCommand {
    public ZeroElevatorEncoder() {
        requires(Robot.elevator);
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        int calibrationValue = Robot.elevator.getRawPosition() - Elevator.lowerLimitEncoderCounts;
        Elevator.calibrationValue = calibrationValue;
        Robot.elevator.runningCalibrationValue = 0;
        Robot.controls.writeIniFile("Elevator", "Encoder_Calibration", String.valueOf(calibrationValue));
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
