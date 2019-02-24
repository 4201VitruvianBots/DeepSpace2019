package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Controls;

import static frc.robot.Robot.intake;

public class TestCargoIntake extends Command {
    double intakeOutput;
    boolean direction;
    boolean testDirection;
    String testName;

    public TestCargoIntake(String testCase, double output) {
        requires(Robot.intake);
        intakeOutput = output;
        testName = testCase;
        setTimeout(0.5);
    }

    @Override
    protected void initialize() {
        // Determine intake direction
        direction = intakeOutput > 0 ? true : false;
    }

    @Override
    protected void execute() {
        intake.setCargoIntakeOutput(intakeOutput);

        if (direction) {
            // Test cargo intake going forward
            testDirection = intake.getMotorOutputPercent() > 0 ? true : false;

        } else {
            // Test cargo intake going backward
            testDirection = intake.getMotorOutputPercent() < 0 ? true : false;
        }
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
        Controls.systemsChecks.put(testName + " Cargo Intake Output", testDirection);
    }

    @Override
    protected void interrupted() { end(); }
}
