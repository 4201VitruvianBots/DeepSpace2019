package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SystemsCheck extends CommandGroup {

    public SystemsCheck() {
        addSequential(new ClearTestResults());

        // Drive Train tests
        addSequential(new TestDriveTrain("Forward", 0.1, 0.1));
        addSequential(new TestDriveTrain("Backwards", -0.1, -0.1));
        addSequential(new TestDriveTrain("Left", -0.1, 0.1));
        addSequential(new TestDriveTrain("Right", +0.1, 0.1));

        // Elevator tests
        addSequential(new TestElevator("Up", 0.1));
        addSequential(new TestElevator("Down", -0.1));

        // Wrist tests
        addSequential(new TestElevator("Up", 0.1));
        addSequential(new TestElevator("Down", -0.1));

        // Cargo Intake tests
        addSequential(new TestCargoIntake("Forward", 0.1));

        // Hatch Ground Intake test
        addSequential(new TestHatchGroundIntake("Forward", 0.1));
    }
}
