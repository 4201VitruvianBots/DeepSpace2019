/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.vitruvianlib.VitruvianLogger.VitruvianLog;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;

/**
 * An example command.  You can replace me with your own command.
 */
public class UpdateElevatorSetpoint extends InstantCommand {
    double alpha = 0.125;
    double lastVoltage = 0;
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

        if(Elevator.controlMode == 1) {
            if (Robot.elevator.getPositionEncoderCounts() > Robot.elevator.upperLimitEncoderCounts)
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) - 0.5;

            // We do this check to make sure co-driver is actually commanding the elevator and not due to minor movement of the joystick.
            // This also prevent an issue where setSetpoint(getSetpoint() + yAxis == 0) continually adds to the setpoint (floating point rounding?)
            if (Math.abs(Robot.m_oi.getXBoxLeftY()) > 0.05) {
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) + (1 * Robot.m_oi.getXBoxLeftY());
            }
        } else {
            double voltage = 0;
            double joystickOutput = Robot.m_oi.getXBoxLeftY();
            if (Math.abs(joystickOutput) > 0.05)
                voltage = 12 * joystickOutput;
            else {
                if(Robot.elevator.getLeftElevatorEncoderHealth() || Robot.elevator.getRightElevatorEncoderHealth())
                    Robot.elevator.setCurrentPositionHold();
                else
                    voltage = 2;
            }
            voltage = Robot.elevator.getLowerLimitSensor() ? Math.max(0, voltage) : voltage;
            voltage = Robot.elevator.getUpperLimitSensor() ? Math.min(0, voltage) : voltage;


            double targetVoltage = alpha * voltage + lastVoltage * (1 - alpha);
            lastVoltage = targetVoltage;

            Robot.elevator.driveOpenLoop(targetVoltage);
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
    }
}
