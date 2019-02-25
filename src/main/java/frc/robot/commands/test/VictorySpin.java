/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class VictorySpin extends Command {
    boolean invert = false;
    int initialState = 0;

    public VictorySpin(double duration) {
        requires(Robot.driveTrain);
        setTimeout(duration);
    }

    public VictorySpin(double duration, boolean invert) {
        requires(Robot.driveTrain);
        setTimeout(duration);
        this.invert = invert;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        initialState = DriveTrain.controlMode;
        DriveTrain.controlMode = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(invert)
            Robot.driveTrain.setMotorPercentOutput(-1, 1);
        else
            Robot.driveTrain.setMotorPercentOutput(1, -1);
    }

    @Override
    protected boolean isFinished(){
        return isTimedOut();
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setMotorPercentOutput(0, 0);
        DriveTrain.controlMode = initialState;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
