/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.UpdateElevatorSetpoint;
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
    private Timer elevatorTimer;
    private double elevatorPreviousTime;
    private double elevatorPreviousError;
    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0;
    private double kS = 0; //Voltage to break static friction
    private double kV = 0; //Voltage to hold constant velocity
    private double kA = 0; //Voltage to hold constant acceleration
    private double maxVelocity = 5;
    private double maxAcceleration = 5;
    public int upperLimitEncoderCounts = 68068; // Silicon, ~65.26 in.
    public int lowerLimitEncoderCounts = 0;
    public static int calibrationValue = 0;
    private int encoderCountsPerInch = 1043;

    private double arbitraryFFUp = 1 / 12;
    private double arbitraryFFDown = 0 / 12;

    public static double elevatorSetPoint = 0;
    public static int controlMode = 0;

    public static boolean initialCalibration = false;
    boolean limitDebounce = false;

    private TalonSRX[] elevatorMotors = {
        new TalonSRX(RobotMap.leftElevatorA),
        new TalonSRX(RobotMap.leftElevatorB),
        new TalonSRX(RobotMap.rightElevatorA),
        new TalonSRX(RobotMap.rightElevatorB),
    };

    private DigitalInput[] limitSwitches = {
        new DigitalInput(RobotMap.elevatorBottom),
        new DigitalInput(RobotMap.elevatorTop),
        new DigitalInput(RobotMap.elevatorMid)
    };

    public Elevator() {
        super("Elevator");

        elevatorMotors[0].setInverted(false);   // Set true for silicon?
        elevatorMotors[1].setInverted(false);   // Set true for silicon?
        elevatorMotors[2].setInverted(true);
        elevatorMotors[3].setInverted(true);

        elevatorMotors[0].setSensorPhase(false); // For whatever reason, Silicon is inverted
        elevatorMotors[2].setSensorPhase(false);

        for (TalonSRX motor : elevatorMotors) {
            motor.configFactoryDefault();
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
            motor.config_kP(0, kP, 30);
            motor.config_kI(0, kI, 30);
            motor.config_kD(0, kD, 30);
            motor.configMotionCruiseVelocity(6000); //7500 is possibly bad
            motor.configMotionAcceleration(13500);
            motor.configContinuousCurrentLimit(30);
            motor.configPeakCurrentLimit(40);
            motor.configPeakCurrentDuration(2000);
            motor.enableCurrentLimit(true);
            motor.configOpenloopRamp(0.6);

            // Fixes watchdog issue?
            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 5);
            motor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5);
            //motor.configForwardSoftLimitEnable(true);
            //motor.configForwardSoftLimitThreshold(upperLimitEncoderCounts);
            //motor.configReverseSoftLimitEnable(true);
            //motor.configReverseSoftLimitThreshold(lowerLimitEncoderCounts);
        }
        elevatorMotors[1].set(ControlMode.Follower, elevatorMotors[0].getDeviceID());
        elevatorMotors[3].set(ControlMode.Follower, elevatorMotors[2].getDeviceID());

//        VitruvianLog elevatorLog = new VitruvianLog("Elevator", 0.5);
//        elevatorLog.addLogField("elevatorPdpLeftCurrent", Controls::getElevatorLeftCurrent);
//        elevatorLog.addLogField("elevatorPdpRightCurrent",  Controls::getElevatorRightCurrent);
//        elevatorLog.addLogField("elevatorTalonLeftCurrent", () -> getMotorCurrent(0));
//        elevatorLog.addLogField("elevatorTalonRightCurrent", () -> getMotorCurrent(1));
////        elevatorLog.addLogField("elevatorTalonLeftVoltage", () -> getMotorVoltage(0));
////        elevatorLog.addLogField("elevatorTalonRightVoltage", () -> getMotorVoltage(1));
////        elevatorLog.addLogField("elevatorTalonLeftOutput", () -> getMotorOutput(0));
////        elevatorLog.addLogField("elevatorTalonRightOutput", () -> getMotorOutput(1));
//        elevatorLog.addLogField("elevatorTalonLeftEncoderCount", () -> elevatorMotors[0].getSelectedSensorPosition());
//        elevatorLog.addLogField("elevatorTalonRightEncoderCount", () -> elevatorMotors[1].getSelectedSensorPosition());
//        VitruvianLogger.getInstance().addLog(elevatorLog);
    }

    public boolean getLimitSwitchState(int limitSwitchIndex){
        return !limitSwitches[limitSwitchIndex].get();
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

    public void zeroEncoder() {
        if(getLimitSwitchState(0)) {
            for (TalonSRX motor : elevatorMotors)
                motor.setSelectedSensorPosition(lowerLimitEncoderCounts,0,0);
            limitDebounce = true;
//        } else if(getLimitSwitchState(1)) {
//            for (TalonSRX motor : elevatorMotors)
//                motor.setSelectedSensorPosition(upperLimitEncoderCounts, 0, 0);
//            limitDebounce = true;
        } else
            limitDebounce = false;
    }

    public void setEncoderPosition(int position) {
        for(TalonSRX motor:elevatorMotors)
            motor.setSelectedSensorPosition(position, 0, 0);
    }

    public int getEncoderPosition(int encoderIndex) {
        return  elevatorMotors[encoderIndex].getSelectedSensorPosition();
    }

    public boolean getEncoderHealth(int encoderIndex) {
        return elevatorMotors[encoderIndex].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public int getPosition() {
        if(getEncoderHealth(0) && getEncoderHealth(2))
            return Math.round((elevatorMotors[0].getSelectedSensorPosition() + elevatorMotors[2].getSelectedSensorPosition())/ 2);
        else if(getEncoderHealth(0)) {
            elevatorMotors[2].set(ControlMode.Follower, elevatorMotors[1].getDeviceID());
            elevatorMotors[3].set(ControlMode.Follower, elevatorMotors[1].getDeviceID());
            return elevatorMotors[0].getSelectedSensorPosition();
        } else if(getEncoderHealth(2)) {
            elevatorMotors[2].set(ControlMode.Follower, elevatorMotors[1].getDeviceID());
            elevatorMotors[3].set(ControlMode.Follower, elevatorMotors[1].getDeviceID());
            return elevatorMotors[2].getSelectedSensorPosition();
        } else //TODO: Make this return an obviously bad value, e.g. 999999999
            return 0;
    }

    public double getHeight() {
        return (double) getPosition() / (double) encoderCountsPerInch;
    }

    public int getVelocity(){
        if(getEncoderHealth(0) && getEncoderHealth(2))
            return Math.round((elevatorMotors[0].getSelectedSensorVelocity() + elevatorMotors[2].getSelectedSensorVelocity()) / 2);
        else if(getEncoderHealth(0))
            return elevatorMotors[0].getSelectedSensorVelocity();
        else if(getEncoderHealth(2))
            return elevatorMotors[2].getSelectedSensorVelocity();
        else //TODO: Make this return an obviously bad value, e.g. 999999999
            return 0;
    }

    public double encoderCountsToInches(double encoderCounts){
        return encoderCounts/encoderCountsPerInch;
    }

    public void setOpenLoopOutput(double voltage){
        elevatorMotors[0].set(ControlMode.PercentOutput, voltage/12, DemandType.ArbitraryFeedForward, voltage >= 0 ? arbitraryFFUp : arbitraryFFDown);
        elevatorMotors[2].set(ControlMode.PercentOutput, voltage/12, DemandType.ArbitraryFeedForward, voltage >= 0 ? arbitraryFFUp : arbitraryFFDown);
    }

    //PID(feedback loop)
    private double setClosedLoopPositionStep(double setPoint) {
        double velocity = (setPoint-elevatorPreviousError)/(elevatorPreviousTime - Timer.getFPGATimestamp());
        double error = setPoint-encoderCountsToInches(getPosition());
        double voltage = 0;
        voltage = kP*error+kD*(error-elevatorPreviousError)/((elevatorPreviousTime - Timer.getFPGATimestamp())- velocity);
        elevatorPreviousError = error;
        elevatorPreviousTime = Timer.getFPGATimestamp();
        return voltage;
    }

    //feed forward loop
    private double setClosedLoopFeedForward(double setPoint) {
        double voltage = 0;
        double error = setPoint-encoderCountsToInches(getPosition()); //gives you the difference between your angle and the desired angle.
        double velocity = encoderCountsToInches(getVelocity());

        if (velocity <= maxVelocity && error >= velocity*velocity/(2*(maxAcceleration))) {
            voltage = kS + kV * velocity + kA * maxAcceleration;
        } else if (error <= velocity*velocity/(2*(maxAcceleration))) {
            voltage = kS + kV * velocity + kA * -maxAcceleration;
        } else {
            voltage = kS + kV * velocity;
        }
        return voltage;
    }

    public void setClosedLoopOutput(double setPoint){
        setOpenLoopOutput(setClosedLoopFeedForward(setPoint) + setClosedLoopPositionStep(setPoint));
    }

    public void setCurrentPositionHold() {
        elevatorMotors[0].set(ControlMode.Position, getPosition());
    }

    public void setIncrementedPosition(double height) {
        double currentPosition = getPosition();
        double encoderCounts = (height * encoderCountsPerInch) + currentPosition;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Elevator", "Setpoint", encoderCounts);

        if (getEncoderHealth(0) && getEncoderHealth(2)) {
            elevatorMotors[0].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
            elevatorMotors[0].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        } else if (getEncoderHealth(0)) {
            elevatorMotors[0].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        } else if (getEncoderHealth(2)) {
            elevatorMotors[2].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        }
    }

    public void setAbsoluteHeight(double height) {
        double encoderCounts = height * encoderCountsPerInch;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Elevator", "Setpoint", encoderCounts);

        if(getEncoderHealth(0) && getEncoderHealth(2)) {
            elevatorMotors[0].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
            elevatorMotors[2].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        } else if(getEncoderHealth(0)) {
            elevatorMotors[0].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        } else if(getEncoderHealth(2)) {
            elevatorMotors[2].set(ControlMode.MotionMagic, encoderCounts, DemandType.ArbitraryFeedForward, encoderCounts > getPosition() ? arbitraryFFUp : arbitraryFFDown);
        }
    }

    public void setElevatorLimitBreak(boolean enable) {
        if(enable) {
            for (TalonSRX motor : elevatorMotors) {
                motor.configContinuousCurrentLimit(38);
            }
            arbitraryFFUp = 0;
            arbitraryFFDown = 0;
        } else {
            for (TalonSRX motor : elevatorMotors) {
                motor.configContinuousCurrentLimit(30);
            }
            arbitraryFFUp = 1 / 12;
            arbitraryFFDown = 0;
        }
    }
    public void updateShuffleBoard() {
        Shuffleboard.putBoolean("Elevator", "Left Encoder Health", getEncoderHealth(0));
        Shuffleboard.putBoolean("Elevator", "Right Encoder Health", getEncoderHealth(2));
//        Shuffleboard.putBoolean("Elevator", "Upper Limit Switch", getLimitSwitchState(1));
//        Shuffleboard.putBoolean("Elevator", "Lower Limit Switch", getLimitSwitchState(0));
//        Shuffleboard.putBoolean("Elevator", "Mid Limit Switch", getLimitSwitchState(2));
        Shuffleboard.putNumber("Elevator", "Elevator Enc Count", getPosition());
        Shuffleboard.putNumber("Elevator", "Elevator Left Enc Count", getEncoderPosition(0));
        Shuffleboard.putNumber("Elevator", "Elevator Right Enc Count", getEncoderPosition(2));
        Shuffleboard.putNumber("Elevator", "Elevator Height", getHeight());
        Shuffleboard.putNumber("Elevator", "Elevator Enc Velocity", getVelocity());
//        Shuffleboard.putNumber("Elevator", "Talon Left Current", getMotorCurrent(0));
//        Shuffleboard.putNumber("Elevator", "Talon Right Current", getMotorCurrent(1));
//        Shuffleboard.putBoolean("Elevator", "isCalibrated", initialCalibration);
        //Shuffleboard.putBoolean("Elevator", "Silicon", !Robot.controls.whichRobot.get());

        Shuffleboard.putNumber("Controls","Elevator Height", getHeight());
        Shuffleboard.putNumber("Controls","Elevator Control Mode", controlMode);
        Shuffleboard.putBoolean("Controls", "Elevator Left Encoder Health", getEncoderHealth(0));
        Shuffleboard.putBoolean("Controls", "Elevator Right Encoder Health", getEncoderHealth(2));

//        SmartDashboard.putBoolean("isElevatorCalibrated", initialCalibration);

        Shuffleboard.putNumber("Elevator", "Control Mode", controlMode);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Elevator Height", getHeight());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new UpdateElevatorSetpoint());
    }
}
