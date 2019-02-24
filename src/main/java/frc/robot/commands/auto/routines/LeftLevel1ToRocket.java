package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.PathfinderRead;
import frc.robot.commands.auto.PathfinderReadLevel1;

public class LeftLevel1ToRocket extends CommandGroup {
    public LeftLevel1ToRocket() {
        addSequential(new PathfinderReadLevel1("getOffLevel1"));
        addSequential(new PathfinderReadLevel1("leftToRocket"));
    }
}
