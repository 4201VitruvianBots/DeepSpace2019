package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.Controls;

import static frc.robot.Robot.driveTrain;

public class TestDriveTrain extends Command {
    double driveTrainLeftOutput, driveTrainRightOutput;
    boolean leftDirection, rightDirection;
    boolean testLeftDirection, testRightDirection, testLeftEncoder, testRightEncoder;
    String testName;

    public TestDriveTrain(String testCase, double leftOutput, double rightOutput) {
        requires(driveTrain);
        driveTrainLeftOutput = leftOutput;
        driveTrainRightOutput = rightOutput;
        testName = testCase;
        setTimeout(0.5);
    }

    @Override
    protected void initialize() {
        driveTrain.zeroEncoderCount();
        // Determine drive train direction
        leftDirection = driveTrainLeftOutput > 0 ? true : false;
        rightDirection = driveTrainRightOutput > 0 ? true : false;
    }

    @Override
    protected void execute() {
        driveTrain.setMotorPercentOutput(driveTrainLeftOutput, driveTrainRightOutput);

        // Test left side of drive train
        if (leftDirection) {
            // Test drive train going forward
            testLeftDirection = driveTrain.driveMotors[1].getMotorOutputPercent() > 0 ? true : false;
            testLeftEncoder = driveTrain.getEncoderCount(0) > 0;
        } else {
            // Test drive train going backward
            testLeftDirection = driveTrain.driveMotors[1].getMotorOutputPercent() < 0 ? true : false;
            testLeftEncoder = driveTrain.getEncoderCount(0) < 0;
        }

        // Test right side of drive train
        if (rightDirection) {
            // Test drive train going forward
            testRightDirection = driveTrain.driveMotors[3].getMotorOutputPercent() > 0 ? true : false;
            testRightEncoder = driveTrain.getEncoderCount(0) > 0;
        } else {
            // Test drive train going backward
            testRightDirection = driveTrain.driveMotors[3].getMotorOutputPercent() < 0 ? true : false;
            testRightEncoder = driveTrain.getEncoderCount(0) < 0;
        }
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Controls.systemsChecks.put(testName +" Drive Train Left Output", testLeftDirection);
        Controls.systemsChecks.put(testName + " Drive Train Right Output", testRightDirection);

        Controls.systemsChecks.put(testName + " Drive Train Right Encoder", testRightEncoder);
        Controls.systemsChecks.put(testName + " Drive Train  Left Encoder", testLeftEncoder);
    }

    @Override
    protected void interrupted() { end(); }
}
