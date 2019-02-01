/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.UpdateElevatorSetpoint;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX[] elevatorMotors = {
        new TalonSRX(RobotMap.leftElevator),
        new TalonSRX(RobotMap.rightElevator),
    };
    private Timer elevatorTimer;
    private double elevatorPreviousTime;
    private double elevatorPreviousError;
    private double kP = 0;
    private double kD = 0;
    private double kS = 0; //Voltage to break static friction
    private double kV = 0; //Voltage to hold constant velocity
    private double kA = 0; //Voltage to hold constant acceleration
    private double maxVelocity = 5;
    private double maxAcceleration = 5;
    public int upperLimitEncoderCounts = 0;
    public int lowerLimitEncoderCounts = 0;
    private int encoderCountsPerInch = 0;

    public static double elevatorSetPoint = 0;
    public static int controlMode = 1;

    DigitalInput[] limitSwitches = {
        new DigitalInput(RobotMap.elevatorBottom),
        new DigitalInput(RobotMap.elevatorTop)
    };

    public Elevator() {
        super("Elevator");

        for (TalonSRX motor : elevatorMotors) {
            motor.configFactoryDefault();
            motor.setInverted(true);
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        }
        elevatorMotors[1].set(ControlMode.Follower, elevatorMotors[0].getDeviceID());
    }

    public boolean getUpperLimitSensor(){
        return limitSwitches[0].get();
    }

    public boolean getLowerLimitSensor(){
        return limitSwitches[1].get();
    }

    public void zeroEncoder() {
        if(getUpperLimitSensor()) {
            for (TalonSRX motor : elevatorMotors)
                motor.setSelectedSensorPosition(upperLimitEncoderCounts, 0, 0);
        } else if(getLowerLimitSensor()) {
            for (TalonSRX motor : elevatorMotors)
                motor.setSelectedSensorPosition(lowerLimitEncoderCounts,0,0);
        }
    }

    public boolean getLeftElevatorEncoderHealth() {
        return elevatorMotors[0].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public boolean getRightElevatorEncoderHealth() {
        return elevatorMotors[1].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public double getPositionEncoderCounts() {
        if(getLeftElevatorEncoderHealth() && getRightElevatorEncoderHealth())
            return (elevatorMotors[0].getSelectedSensorPosition() + elevatorMotors[1].getSelectedSensorPosition()) / 2;
        else if(getLeftElevatorEncoderHealth())
            return elevatorMotors[0].getSelectedSensorPosition();
        else if(getLeftElevatorEncoderHealth())
            return elevatorMotors[1].getSelectedSensorPosition();
        else //TODO: Make this return an obviously bad value, e.g. 999999999
            return 0;
    }

    public double getVelocityEncoderCounts(){
        if(getLeftElevatorEncoderHealth() && getRightElevatorEncoderHealth())
            return (elevatorMotors[0].getSelectedSensorVelocity() + elevatorMotors[1].getSelectedSensorVelocity()) / 2;
        else if(getLeftElevatorEncoderHealth())
            return elevatorMotors[0].getSelectedSensorVelocity();
        else if(getLeftElevatorEncoderHealth())
            return elevatorMotors[1].getSelectedSensorVelocity();
        else //TODO: Make this return an obviously bad value, e.g. 999999999
            return 0;
    }

    public double encoderCountsToInches(double encoderCounts){
        return encoderCounts/encoderCountsPerInch;
    }

    public void driveOpenLoop(double voltage){
        elevatorMotors[0].set(ControlMode.PercentOutput, voltage/12);
    }

    //PID(feedback loop)
    private double setClosedLoopPositionStep(double setPoint) {
        double velocity = (setPoint-elevatorPreviousError)/(elevatorPreviousTime -elevatorTimer.getFPGATimestamp());
        double error = setPoint-encoderCountsToInches(getPositionEncoderCounts());
        double voltage = 0;
        voltage = kP*error+kD*(error-elevatorPreviousError)/((elevatorPreviousTime -elevatorTimer.getFPGATimestamp())- velocity);
        elevatorPreviousError = error;
        elevatorPreviousTime = elevatorTimer.getFPGATimestamp();
        return voltage;
    }

    //feed forward loop
    private double setClosedLoopFeedForward(double setPoint) {
        double voltage = 0;
        double error = setPoint-encoderCountsToInches(getPositionEncoderCounts()); //gives you the difference between your angle and the desired angle.
        double velocity = encoderCountsToInches(getVelocityEncoderCounts());
        if (velocity <= maxVelocity && error >= velocity*velocity/(2*(maxAcceleration))) {
            voltage = kS + kV * velocity + kA * maxAcceleration;
        } else if (error <= velocity*velocity/(2*(maxAcceleration))) {
            voltage = kS + kV * velocity + kA * -maxAcceleration;
        } else {
            voltage = kS + kV * velocity;
        }
        return voltage;
    }


    public void setClosedLoop(double setPoint){
        driveOpenLoop(setClosedLoopFeedForward(setPoint) + setClosedLoopPositionStep(setPoint));
    }

    public void updateSmartDashboard() {
        //Shuffleboard.putBoolean("Vision","IsValidTarget", isValidTarget());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new UpdateElevatorSetpoint());
    }
}
