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
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.UpdateWristSetpoint;

import javax.naming.ldap.Control;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Wrist extends PIDSubsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    static double kP = 0.03;
    static double kI = 0;
    static double kD = 0.00;
    static double kF = 0;

    double encoderCountRatio = 1; //Ratio for angle over encoder count.

    public static int controlMode = 0;
    private TalonSRX wristMotor = new TalonSRX(RobotMap.wristMotor);

    public Wrist() {
        super("Wrist", kP, kI, kD);

        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(false);

        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public int getEncoderCount() {
        return wristMotor.getSelectedSensorPosition();
    }

    public double getEncoderVelocity() {
        return wristMotor.getSelectedSensorVelocity();
    }

    // Using the pulse width measurement, check if the encoders are healthy
    public boolean isEncoderHealthy() {
        return wristMotor.getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public double getAngle() {
        return getEncoderCount() * encoderCountRatio;
    }

    public ControlMode getTalonControlMode() {
        return wristMotor.getControlMode();
    }

    public void setMotorState(boolean state) {
        wristMotor.setNeutralMode((state) ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void setDirectOutput(double output) {
        if (output == 0) {
            if(isEncoderHealthy())
                wristMotor.set(ControlMode.Position, getEncoderCount());
            else
                wristMotor.set(ControlMode.PercentOutput, 0);
        } else
            wristMotor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double returnPIDInput() {
        return getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        // TODO Auto-generated method stub
        wristMotor.set(ControlMode.PercentOutput, output);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Wrist Encoder Count", getEncoderCount());
        SmartDashboard.putNumber("Wrist Encoder Velocity", getEncoderVelocity());
        SmartDashboard.putBoolean("Wrist Encoder Health", isEncoderHealthy());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new UpdateWristSetpoint());
    }
}
