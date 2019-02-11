/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example command.  You can replace me with your own command.
 */
public class UpdateElevatorSetpoint extends Command {
    double alpha = 0.125;
    static double lastVoltage = 0;

    public static boolean override = false;

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
        double joystickOutput = Robot.m_oi.getXBoxLeftY();

        if(Elevator.controlMode == 1 && !override) {
            /*
            if (Robot.elevator.getPosition() > Robot.elevator.upperLimitEncoderCounts)
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) - 0.5;

            // We do this check to make sure co-driver is actually commanding the elevator and not due to minor movement of the joystick.
            // This also prevent an issue where setSetpoint(getSetpoint() + yAxis == 0) continually adds to the setpoint (floating point rounding?)
            if (Math.abs(Robot.m_oi.getXBoxLeftY()) > 0.05) {
                Elevator.elevatorSetPoint = Robot.elevator.encoderCountsToInches(Robot.elevator.upperLimitEncoderCounts) + (1 * Robot.m_oi.getXBoxLeftY());
            }
            */
            if(Math.abs(joystickOutput) > 0.05)
                Robot.elevator.setIncrementedHeight(joystickOutput * 6);
        } else {
            double voltage = 0;
            if (Math.abs(joystickOutput) > 0.05)
                voltage = 12 * joystickOutput;
            else {
                if(Robot.elevator.getEncoderHealth(0) || Robot.elevator.getEncoderHealth(1))
                    Robot.elevator.setCurrentPositionHold();
                else
                    voltage = 2;
            }
            //voltage = Robot.elevator.getLowerLimitSensor() ? Math.max(0, voltage) : voltage;
            //voltage = Robot.elevator.getUpperLimitSensor() ? Math.min(0, voltage) : voltage;


            double targetVoltage = alpha * voltage + lastVoltage * (1 - alpha);
            lastVoltage = targetVoltage;

            // Debugging
            Shuffleboard.putNumber("Elevator", "Open-Loop Target Voltage", targetVoltage);

            Robot.elevator.setOpenLoopOutput(targetVoltage);
        }

        // TODO: Figure out why this only works on the trigger and not as a command on other xBox buttons
        if(Robot.m_oi.xBoxLeftTrigger.get()) {
            Shuffleboard.putBoolean("Elevator", "Test Button", true);
            Robot.elevator.setOpenLoopOutput(Shuffleboard.getNumber("Elevator", "Test Voltage", 0));
        } else
            Shuffleboard.putBoolean("Elevator", "Test Button", false);

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
