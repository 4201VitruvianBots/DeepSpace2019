package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.Controls;

import static frc.robot.Robot.driveTrain;
import static frc.robot.Robot.wrist;

public class TestWrist extends Command {
    double wristOutput;
    boolean direction;
    boolean testDirection, testWristEncoder;
    String testName;

    public TestWrist(String testCase, double output) {
        requires(wrist);
        wristOutput = output;
        testName = testCase;
        setTimeout(0.5);
    }

    @Override
    protected void initialize() {
        wrist.zeroEncoder();
        // Determine wrist direction
        direction = wristOutput> 0 ? true : false;
    }

    @Override
    protected void execute() {
        wrist.setDirectOutput(wristOutput);

        if (direction) {
            // Test wrist going forward
            testDirection = wrist.getMotorOutputPercent() > 0 ? true : false;
            testWristEncoder = wrist.getPosition() > 0 ? true : false;
        } else {
            // Test wrist going backward
            testDirection = wrist.getMotorOutputPercent() < 0 ? true : false;
            testWristEncoder = wrist.getPosition() < 0 ? true : false;
        }
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Controls.systemsChecks.put(testName + " Wrist Output", testDirection);
        Controls.systemsChecks.put(testName + " Wrist Encoder", testWristEncoder);

    }

    @Override
    protected void interrupted() { end(); }
}
