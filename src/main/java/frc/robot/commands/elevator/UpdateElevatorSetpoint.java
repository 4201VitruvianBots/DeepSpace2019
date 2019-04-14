/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class UpdateElevatorSetpoint extends Command {
    Timer stopwatch = new Timer();
    boolean mutex = false;

    public UpdateElevatorSetpoint() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double joystickInput = Math.abs(Robot.m_oi.getXBoxLeftY()) > 0.05 ? Robot.m_oi.getXBoxLeftY() : 0;
        double joystickOutput = 0.5 * (joystickInput + Math.pow(joystickInput, 3));

        if(!Elevator.initialCalibration) {
             if(Robot.elevator.getLimitSwitchState(0) || Robot.elevator.getLimitSwitchState(1)) {
                 Elevator.initialCalibration = true;
                 //Elevator.controlMode = 1;
             }
        }

        if(Elevator.controlMode == 1) {
            /*
            if (Robot.elevator.getPosition() > Robot.elevator.upperLimitEncoderCounts)
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) - 0.5;

            // We do this check to make sure co-driver is actually commanding the elevator and not due to minor movement of the joystick.
            // This also prevent an issue where setSetpoint(getSetpoint() + yAxis == 0) continually adds to the setpoint (floating point rounding?)
            if (Math.abs(Robot.m_oi.getXBoxLeftY()) > 0.05) {
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) + (1 * Robot.m_oi.getXBoxLeftY());
            }
            */
            double setpoint = joystickOutput * 6;

            // TODO: Change this logic to use limit switches when they are fixed
            if(setpoint == 0 && Robot.elevator.getHeight() < 0.1 || setpoint == 64 && Robot.elevator.getHeight() > 63.9)
                Robot.m_oi.enableXBoxRumbleTimed(0.2);

            Robot.elevator.setIncrementedPosition(setpoint);
            
            boolean trip = false;
            for(int i = 0; i < 2; i++)
            	if(Robot.elevator.getMotorCurrent(i) > 25)
            		trip = true;
            
            if(trip) {
        		if(!mutex) {
        			mutex = true;
        			stopwatch.reset();
        			stopwatch.start();
        		}
        		if(stopwatch.get() > 1) {
                	mutex = false;
                	stopwatch.stop();

                    Robot.elevator.setAbsoluteHeight(Robot.elevator.getHeight());
        		}
            } else if(mutex) {
            	mutex = false;
            	stopwatch.stop();
            }
            
        } else {
            double voltage = 12 * joystickOutput;
            //if(Robot.elevator.getEncoderHealth(0) || Robot.elevator.getEncoderHealth(1))
            //    Robot.elevator.setCurrentPositionHold();
            //else if(Robot.m_oi.xBoxPOVButtons[0].get())
            //    voltage = 2;

            // TODO: Uncomment once limit switches are implemented
            /*if(Robot.elevator.getLimitSwitchState(0) || Robot.elevator.getLimitSwitchState(1)) {
                voltage = 0;
                Robot.m_oi.setXBoxRumble(0.8);
            } else
                Robot.m_oi.setXBoxRumble(0);*/

            Robot.elevator.setOpenLoopOutput(voltage);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
