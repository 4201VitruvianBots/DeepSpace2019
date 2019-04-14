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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.wrist.UpdateWristSetpoint;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Wrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    static double kP = 2;
    static double kI = 0;
    static double kD = 0;
    static double kF = 0;
    static double arbitraryFF = 0;
                                                      //5026 135 4096 * 0.375 * (72/22)
                                                      //5802 1170 4096 * 0.375 * (72/22)
                                                      //5632; // 5802 165 degrees, 4096 * (165/360) * 3
                                                      //-682; // -682 -20 degrees, 4096 * (1/18) * 3
    public static int upperLimitEncoderCounts = 5026; // 5026 135 degrees, 4096 * (135/360) * (72/22)
    public static int lowerLimitEncoderCounts = -744; // -744 -20 degrees, 4096 * (-20/360) * (72/22)
    public static int calibrationValue = 0;
    double encoderCountsPerAngle = 37.236;  // 34.133

    public static int controlMode = 1;
    static boolean limitDebounce = false;
    private TalonSRX wristMotor = new TalonSRX(RobotMap.wristMotor);

    private DigitalInput[] limitSwitches = {
        new DigitalInput(RobotMap.wristBottom),
        new DigitalInput(RobotMap.wristTop)
    };

    public Wrist() {
        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
        wristMotor.setSensorPhase(false);
        wristMotor.configContinuousCurrentLimit(20);
        wristMotor.configPeakCurrentLimit(30);
        wristMotor.configPeakCurrentDuration(2000);
        wristMotor.enableCurrentLimit(true);
//        wristMotor.configVoltageCompSaturation(12);
//        wristMotor.enableVoltageCompensation(true);
        wristMotor.configForwardSoftLimitEnable(false);
        wristMotor.configReverseSoftLimitEnable(false);

        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        wristMotor.config_kP(0, kP, 30);
        wristMotor.config_kI(0, kI, 30);
        wristMotor.config_kD(0, kD, 30);
        wristMotor.configClosedloopRamp(0.1, 100);
    }

    public int getPosition() {
        return wristMotor.getSelectedSensorPosition() + calibrationValue;
    }

    public double getVelocity() {
        return wristMotor.getSelectedSensorVelocity();
    }

    public ControlMode getTalonControlMode() {
        return wristMotor.getControlMode();
    }

    // Using the pulse width measurement, check if the encoders are healthy
    public boolean getEncoderHealthy() {
        return wristMotor.getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public boolean getLimitSwitchState(int limitSwitchIndex){
        return !limitSwitches[limitSwitchIndex].get();
    }

    public void zeroEncoder() {
        if(getLimitSwitchState(0)) {
            wristMotor.setSelectedSensorPosition(lowerLimitEncoderCounts, 0, 0);
            limitDebounce = true;
        } else if(getLimitSwitchState(1)) {
            wristMotor.setSelectedSensorPosition(upperLimitEncoderCounts, 0, 0);
            limitDebounce = true;
        } else
            limitDebounce = false;
    }

    public void setEncoderPosition(int position) {
        wristMotor.setSelectedSensorPosition(position, 0, 0);
    }

    public double getAngle() {
        return getPosition() / encoderCountsPerAngle;
    }
    
    public double getOutputCurrent() {
    	return wristMotor.getOutputCurrent();
    }

    public void setDirectOutput(double output) {
        if (output == 0) {
            if(getEncoderHealthy())
                wristMotor.set(ControlMode.Position, getPosition(), DemandType.ArbitraryFeedForward, arbitraryFF);
            else
                wristMotor.set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, arbitraryFF);
        } else
            wristMotor.set(ControlMode.PercentOutput, output, DemandType.ArbitraryFeedForward, arbitraryFF);
    }
    
    public void setIncrementedPosition(double angle) {
        double currentPosition = getPosition();
        double encoderCounts = angle * encoderCountsPerAngle + currentPosition;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Wrist", "Setpoint", encoderCounts);
        wristMotor.set(ControlMode.Position, encoderCounts, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void setAbsolutePosition(double angle) {
        double encoderCounts = angle * encoderCountsPerAngle;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Wrist", "Setpoint", encoderCounts);
        wristMotor.set(ControlMode.Position, encoderCounts, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void updateShuffleboard() {
        Shuffleboard.putNumber("Wrist","Encoder Count", getPosition());
        Shuffleboard.putNumber("Wrist","Angle", getAngle());
//        Shuffleboard.putNumber("Wrist","Encoder Velocity", getVelocity());
        Shuffleboard.putNumber("Wrist","Control Mode", controlMode);
        Shuffleboard.putBoolean("Wrist","Encoder Health", getEncoderHealthy());
//        Shuffleboard.putBoolean("Wrist","Lower Limit Switch", getLimitSwitchState(0));
//        Shuffleboard.putBoolean("Wrist","Upper Limit Switch", getLimitSwitchState(1));

        Shuffleboard.putNumber("Controls","Wrist Angle", getAngle());
        Shuffleboard.putNumber("Controls","Wrist Control Mode", controlMode);

        Shuffleboard.putBoolean("Controls","Wrist Encoder Health", getEncoderHealthy());
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Wrist Angle", getAngle());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new UpdateWristSetpoint());
    }
}
