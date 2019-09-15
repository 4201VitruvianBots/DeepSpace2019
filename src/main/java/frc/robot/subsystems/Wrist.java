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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PDP;
import frc.robot.commands.wrist.UpdateWristSetpoint;
import frc.vitruvianlib.VitruvianLogger.VitruvianLog;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;
import frc.vitruvianlib.drivers.CachedTalonSRX;
import frc.vitruvianlib.driverstation.Shuffleboard;

import static frc.robot.subsystems.Controls.getPdpCurrent;

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
    public static int upperLimitEncoderCounts = 5772; // 5400 155 degrees, 4096 * (155/360) * (72/22)
    public static int lowerLimitEncoderCounts = -372; // -372 -10 degrees, 4096 * (-10/360) * (72/22)
    public static int zeroOffset = 0;
    double encoderCountsPerAngle = 37.236;            // 1 degree, 4096 * (1/360) * (72/22)
    
    public int controlMode = 1;
    static boolean limitDebounce = false;
    private CachedTalonSRX wristMotor = new CachedTalonSRX(RobotMap.wristMotor);

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
        wristMotor.configVoltageCompSaturation(12);
        wristMotor.enableVoltageCompensation(true);
        wristMotor.configForwardSoftLimitEnable(false);
        wristMotor.configReverseSoftLimitEnable(false);
        wristMotor.configOpenloopRamp(0.1);
        wristMotor.configClosedloopRamp(0.1);

        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        wristMotor.config_kP(0, kP, 30);
        wristMotor.config_kI(0, kI, 30);
        wristMotor.config_kD(0, kD, 30);
        
//        VitruvianLog wristLog = new VitruvianLog("Wrist", 0.5);
//        wristLog.addLogField("wristPdpCurrent", () ->  getPdpCurrent(PDP.WRIST));
//        wristLog.addLogField("wristTalonCurrent", () -> wristMotor.getOutputCurrent());
//        VitruvianLogger.getInstance().addLog(wristLog);
    }

    public int getPosition() {
        return wristMotor.getSelectedSensorPosition() - zeroOffset;
    }

    public double getVelocity() {
        return wristMotor.getSelectedSensorVelocity();
    }

    public double getOutputCurrent() {
        return  wristMotor.getOutputCurrent();
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

    public void setEncoderZeroOffset(double zeroAngle) {
    	zeroOffset = -wristMotor.getSelectedSensorPosition() + (int) Math.round(zeroAngle * encoderCountsPerAngle);
        Robot.controls.writeIniFile("Wrist", "Encoder_Calibration", String.valueOf(zeroOffset));
    }

    public void setEncoderPosition(int position) {
        wristMotor.setSelectedSensorPosition(position, 0, 0);
    }

    public double getAngle() {
        return getPosition() / encoderCountsPerAngle;
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
    
    public void setIncrementedAngle(double angle) {
        double currentPosition = getPosition();
        double encoderCounts = angle * encoderCountsPerAngle + currentPosition + zeroOffset;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Wrist", "Setpoint", encoderCounts);
        wristMotor.set(ControlMode.Position, encoderCounts, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void setAbsoluteAngle(double angle) {
        double encoderCounts = angle * encoderCountsPerAngle + zeroOffset;

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
