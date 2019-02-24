package frc.robot.commands.test.systemscheck;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.util.Controls;

public class ClearTestResults extends InstantCommand {

    public ClearTestResults() {

    }

    @Override
    protected void initialize() {
        Controls.systemsChecks.clear();
    }

    @Override
    protected void execute() {

    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() { end(); }
}
