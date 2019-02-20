package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ManualElevatorGround;
import frc.robot.commands.wrist.ManualWristUp;

public class SaveMe extends CommandGroup {
    public SaveMe() {
        addSequential(new KillAll());
        addParallel(new ManualWristUp());
        addSequential(new ManualElevatorGround());
    }
}
