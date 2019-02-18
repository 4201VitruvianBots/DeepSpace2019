/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class SetElevatorVoltage extends Command {
   double voltage;

    public SetElevatorVoltage() {
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        voltage = Shuffleboard.getNumber("Elevator", "Test Voltage", 0);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.elevator.setOpenLoopOutput(voltage);
    }

    @Override
    protected boolean isFinished(){
        return false;
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.elevator.setOpenLoopOutput(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
