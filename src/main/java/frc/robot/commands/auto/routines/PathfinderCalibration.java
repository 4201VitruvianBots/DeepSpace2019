package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.PathfinderRead;

public class PathfinderCalibration extends CommandGroup {
    public PathfinderCalibration() {
        addSequential(new PathfinderRead("driveCalibration"));
    }
}
