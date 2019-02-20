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
        int leftCalibrationValue = Elevator.lowerLimitEncoderCounts - Robot.elevator.getPosition(0);
        int rightCalibrationValue = Elevator.lowerLimitEncoderCounts - Robot.elevator.getPosition(1);
        Elevator.leftCalibrationValue = leftCalibrationValue;
        Elevator.rightCalibrationValue = rightCalibrationValue;
//        Robot.elevator.runningCalibrationValue = 0;
//        Robot.elevator.setPosition(0, Robot.elevator.getPosition(0) + leftCalibrationValue);
//        Robot.elevator.setPosition(1, Robot.elevator.getPosition(1) + rightCalibrationValue);
        Robot.controls.writeIniFile("Elevator", "Encoder_Left_Calibration", String.valueOf(leftCalibrationValue));
        Robot.controls.writeIniFile("Elevator", "Encoder_Right_Calibration", String.valueOf(rightCalibrationValue));
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
