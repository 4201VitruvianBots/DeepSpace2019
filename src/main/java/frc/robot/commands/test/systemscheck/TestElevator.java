package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.Controls;

import static frc.robot.Robot.elevator;

public class TestElevator extends Command {
    double elevatorOutput;
    boolean direction;
    boolean testDirection, testLeftEncoder, testRightEncoder;
    String testName;

    public TestElevator(String testCase, double output) {
        requires(elevator);
        elevatorOutput = output;
        testName = testCase;
        setTimeout(0.5);
    }

    @Override
    protected void initialize() {
        elevator.zeroEncoder();
        // Determine elevator direction
        direction = elevatorOutput > 0 ? true : false;
    }

    @Override
    protected void execute() {
        elevator.setOpenLoopOutput(elevatorOutput);

        if (direction) {
            // Test elevator going forward
            testDirection = elevator.getMotorOutput(1) > 0 ? true : false;
            testLeftEncoder = elevator.getEncoderPosition(0) > 0 ? true : false;
            testRightEncoder = elevator.getEncoderPosition(1) > 0 ? true : false;
        } else {
            // Test elevator going backward
            testDirection = elevator.getMotorOutput(1) < 0 ? true : false;
            testRightEncoder = elevator.getEncoderPosition(0) < 0 ? true : false;
            testLeftEncoder = elevator.getEncoderPosition(1) < 0 ? true : false;
        }
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Controls.systemsChecks.put(testName + " Elevator Open Loop Output", testDirection);
        Controls.systemsChecks.put(testName + " Elevator Open Loop Left Encoder", testLeftEncoder);
        Controls.systemsChecks.put(testName + " Elevator Open Loop Right Encoder", testRightEncoder);
    }

    @Override
    protected void interrupted() { end(); }
}
