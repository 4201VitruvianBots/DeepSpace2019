/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.drivetrain.SetArcadeDriveVelocity;
import frc.robot.util.Controls;
import frc.vitruvianlib.VitruvianLogger.VitruvianLog;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private TalonSRX[] driveMotors = {
        new TalonSRX(RobotMap.leftFrontDriveMotor),
        new TalonSRX(RobotMap.leftRearDriveMotor),
        new TalonSRX(RobotMap.rightFrontDriveMotor),
        new TalonSRX(RobotMap.rightRearDriveMotor),
    };

    DoubleSolenoid driveTrainShifters = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.driveTrainShifterForward, RobotMap.driveTrainShifterReverse);
    public AHRS navX = new AHRS(SerialPort.Port.kMXP);

    public static int controlMode = 0;

    public DriveTrain() {
        super("DriveTrain");

        for (TalonSRX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.config_kP(0, 0.25, 30);
            motor.config_kI(0, 0, 30);
            motor.config_kD(0, 10, 30);
            motor.config_kF(0, 1023.0 / 72000.0, 30);
            motor.configVoltageCompSaturation(12);
            motor.enableVoltageCompensation(true);
            motor.configContinuousCurrentLimit(30);
            motor.configPeakCurrentLimit(40);
            motor.configPeakCurrentDuration(2000);
            motor.enableCurrentLimit(true);
            motor.configOpenloopRamp(0.6);
            motor.setNeutralMode(NeutralMode.Coast);
        }

        driveMotors[0].setInverted(true);
        driveMotors[1].setInverted(true);
        driveMotors[2].setInverted(false);
        driveMotors[3].setInverted(false);

        driveMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        driveMotors[2].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());


//        VitruvianLog drivetrainLog = new VitruvianLog("DriveTrain", 0.5);
//        drivetrainLog.addLogField("drivetrainPdpLeftFrontCurrent", () -> Controls.pdp.getCurrent(RobotMap.pdpChannelDriveTrainLeftForward));
//        drivetrainLog.addLogField("drivetrainPdpLeftRearCurrent",  () -> Controls.pdp.getCurrent(RobotMap.pdpChannelDriveTrainLeftReverse));
//        drivetrainLog.addLogField("drivetrainPdpRightFrontCurrent", () -> Controls.pdp.getCurrent(RobotMap.pdpChannelDriveTrainRightForward));
//        drivetrainLog.addLogField("drivetrainPdpRightRearCurrent", () -> Controls.pdp.getCurrent(RobotMap.pdpChannelDriveTrainRightReverse));
//        drivetrainLog.addLogField("drivetrainTalonLeftFrontCurrent", () -> driveMotors[0].getOutputCurrent());
//        drivetrainLog.addLogField("drivetrainTalonLeftRearCurrent", () -> driveMotors[1].getOutputCurrent());
//        drivetrainLog.addLogField("drivetrainTalonRightFrontCurrent", () -> driveMotors[2].getOutputCurrent());
//        drivetrainLog.addLogField("drivetrainTalonRightRearCurrent", () -> driveMotors[3].getOutputCurrent());
//        VitruvianLogger.getInstance().addLog(drivetrainLog);
    }

    public int getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getEncoderVelocity(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorVelocity();
    }


    public void zeroEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
    }

    public ControlMode getTalonControlMode() {
        return driveMotors[0].getControlMode();
    }
    // Using the pulse width measurement, check if the encoders are healthy

    public boolean getEncoderHealth(int encoderIndex) {
        return driveMotors[encoderIndex].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public void setDriveMotorsState(boolean state) {
        for (TalonSRX driveMotor : driveMotors)
            driveMotor.setNeutralMode((state) ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        if(rightPWM > 1.0) {
            leftPWM -= rightPWM - 1.0;
            rightPWM = 1.0;
        } else if(rightPWM < -1.0) {
            leftPWM -= rightPWM + 1.0;
            rightPWM = -1.0;
        } else if(leftPWM > 1.0) {
            rightPWM -= leftPWM - 1.0;
            leftPWM = 1.0;
        } else if(leftPWM < -1.0) {
            rightPWM -= leftPWM + 1.0;
            leftPWM = -1.0;
        }

        setMotorPercentOutput(leftPWM, rightPWM);
    }

    public void setMotorTankDrive(double leftOutput, double rightOutput) {
        setMotorPercentOutput(leftOutput, rightOutput);
    }

    public void setArcadeDriveVelocity(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        if (rightPWM > 1.0) {
            leftPWM -= rightPWM - 1.0;
            rightPWM = 1.0;
        } else if (rightPWM < -1.0) {
            leftPWM -= rightPWM + 1.0;
            rightPWM = -1.0;
        } else if (leftPWM > 1.0) {
            rightPWM -= leftPWM - 1.0;
            leftPWM = 1.0;
        } else if (leftPWM < -1.0) {
            rightPWM -= leftPWM + 1.0;
            leftPWM = -1.0;
        }

        setMotorVelocityOutput(leftPWM, rightPWM);
    }

    public void setMotorVelocityOutput(double leftOutput, double rightOutput) {
        //TODO: Update values to match robot with full load.
        double k_maxVelocity = getDriveShifterStatus() ? 8789 : 18555;  // in encoder units/sec

        // TODO: Normalize this
        double leftVelocity = leftOutput * k_maxVelocity;
        double rightVelocity = rightOutput * k_maxVelocity;

        leftVelocity = (leftVelocity > k_maxVelocity) ? k_maxVelocity : (leftVelocity < -k_maxVelocity) ? -k_maxVelocity: leftVelocity;
        rightVelocity = (rightVelocity > k_maxVelocity) ? k_maxVelocity : (rightVelocity < -k_maxVelocity) ? -k_maxVelocity: rightVelocity;

        driveMotors[0].set(ControlMode.Velocity, leftVelocity);
        driveMotors[2].set(ControlMode.Velocity, rightVelocity);
    }

    public void setMotorPercentOutput(double leftOutput, double rightOutput) {
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
    }

    public boolean getDriveShifterStatus() {
        return (driveTrainShifters.get() == DoubleSolenoid.Value.kForward) ? true : false;
    }

    public void setDriveShifterStatus(boolean state) {
        driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void updateShuffleboard() {
//        Shuffleboard.putBoolean("DriveTrain", "Left Encoder Health", getEncoderHealth(0));
//        Shuffleboard.putBoolean("DriveTrain", "Right Encoder Health", getEncoderHealth(2));
//
//        Shuffleboard.putNumber("DriveTrain", "Left Encoder Count", getEncoderCount(0));
//        Shuffleboard.putNumber("DriveTrain", "Right Encoder Count", getEncoderCount(2));
//
//        Shuffleboard.putNumber("Controls", "DriveTrain Control Mode", controlMode);
//
//        //SmartDashboard.putNumber("NavX Temp (C)", navX.getTempC());

        Shuffleboard.putNumber("DriveTrain","Robot Angle", navX.getAngle());
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Angle", navX.getAngle());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //defaultCommand = new SetArcadeDriveVelocity();
        setDefaultCommand(new SetArcadeDrive());
    }
}
