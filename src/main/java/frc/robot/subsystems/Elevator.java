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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.UpdateElevatorSetpoint;
import frc.robot.util.Controls;
import frc.vitruvianlib.VitruvianLogger.VitruvianLog;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;
import frc.vitruvianlib.driverstation.Shuffleboard;

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
    public static int controlMode = 0;

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
        elevatorMotors[0].setSensorPhase(true);
        elevatorMotors[1].set(ControlMode.Follower, elevatorMotors[0].getDeviceID());

        VitruvianLog elevatorLog = new VitruvianLog("Elevator", 0.5);
        elevatorLog.addLogField("elevatorPdpLeftCurrent", Controls::getElevatorLeftCurrent);
        elevatorLog.addLogField("elevatorPdpRightCurrent",  Controls::getElevatorRightCurrent);
        elevatorLog.addLogField("elevatorTalonLeftCurrent", () -> getMotorCurrent(0));
        elevatorLog.addLogField("elevatorTalonRightCurrent", () -> getMotorCurrent(1));
        elevatorLog.addLogField("elevatorTalonLeftVoltage", () -> getMotorVoltage(0));
        elevatorLog.addLogField("elevatorTalonRightVoltage", () -> getMotorVoltage(1));
        elevatorLog.addLogField("elevatorTalonLeftOutput", () -> getMotorOutput(0));
        elevatorLog.addLogField("elevatorTalonRightOutput", () -> getMotorOutput(1));
        elevatorLog.addLogField("elevatorTalonLeftEncoderCount", () -> elevatorMotors[0].getSelectedSensorPosition());
        elevatorLog.addLogField("elevatorTalonRightEncoderCount", () -> elevatorMotors[1].getSelectedSensorPosition());
        VitruvianLogger.getInstance().addLog(elevatorLog);
    }

    public boolean getUpperLimitSensor(){
        return !limitSwitches[0].get();
    }

    public boolean getLowerLimitSensor(){
        return !limitSwitches[1].get();
    }

    public double getMotorCurrent(int motorIndex) {
        return elevatorMotors[motorIndex].getOutputCurrent();
    }

    public double getMotorVoltage(int motorIndex) {
        return elevatorMotors[motorIndex].getMotorOutputVoltage();
    }

    public double getMotorOutput(int motorIndex) {
        return elevatorMotors[motorIndex].getMotorOutputPercent();
    }

    public void resetEncoderCount() {
        for(TalonSRX talon:elevatorMotors)
            talon.setSelectedSensorPosition(0);
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

    public void setCurrentPositionHold() {
        elevatorMotors[0].set(ControlMode.Position, getPositionEncoderCounts());
    }

    public void setClosedLoop(double setPoint){
        driveOpenLoop(setClosedLoopFeedForward(setPoint) + setClosedLoopPositionStep(setPoint));
    }

    public void updateSmartDashboard() {
        Shuffleboard.putBoolean("Elevator", "Left Encoder Health", getLeftElevatorEncoderHealth());
        Shuffleboard.putBoolean("Elevator", "Right Encoder Health", getRightElevatorEncoderHealth());
        Shuffleboard.putBoolean("Elevator", "Upper Limit Switch", getUpperLimitSensor());
        Shuffleboard.putBoolean("Elevator", "Lower Limit Switch", getLowerLimitSensor());
        Shuffleboard.putNumber("Elevator", "Elevator Enc Count", getPositionEncoderCounts());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new UpdateElevatorSetpoint());
    }
}
