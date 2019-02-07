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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.UpdateWristSetpoint;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Wrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    static double kP = 0.03;
    static double kI = 0;
    static double kD = 0;
    static double kF = 0;
    static double arbitraryFF = 0;

    double upperLimitEncoderCounts;
    double lowerLimitEncoderCounts;
    double encoderCountsToAngle;
    
    
    public static int controlMode = 0;
    private TalonSRX wristMotor = new TalonSRX(RobotMap.wristMotor);

    public Wrist() {
        wristMotor.configFactoryDefault();
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(false);

        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public int getPosition() {
        return wristMotor.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return wristMotor.getSelectedSensorVelocity();
    }

    public ControlMode getTalonControlMode() {
        return wristMotor.getControlMode();
    }


    // Using the pulse width measurement, check if the encoders are healthy
    public boolean isEncoderHealthy() {
        return wristMotor.getSensorCollection().getPulseWidthRiseToFallUs() != 0;
    }

    public double getAngle() {
        return getPosition() * encoderCountsToAngle;
    }
    public void setMotorState(boolean state) {
        wristMotor.setNeutralMode((state) ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void setDirectOutput(double output) {
        if (output == 0) {
            if(isEncoderHealthy())
                wristMotor.set(ControlMode.Position, getPosition());
            else
                wristMotor.set(ControlMode.PercentOutput, 0);
        } else
            wristMotor.set(ControlMode.PercentOutput, output);
    }
    
    public void setIncrementedPosition(double angle) {
        double angleToEncderCounts = 1;
        double currentPosition = getPosition();
        double encoderCounts = angle * angleToEncderCounts + currentPosition;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Elevator", "Setpoint", encoderCounts);
        wristMotor.set(ControlMode.Position, encoderCounts, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void setAbsolutePosition(double angle) {
        double hieghtToEncoderCounts = 1;
        double encoderCounts = angle * hieghtToEncoderCounts;

        encoderCounts = encoderCounts > upperLimitEncoderCounts ? upperLimitEncoderCounts : encoderCounts;
        encoderCounts = encoderCounts < lowerLimitEncoderCounts ? lowerLimitEncoderCounts : encoderCounts;

        Shuffleboard.putNumber("Elevator", "Setpoint", encoderCounts);
        wristMotor.set(ControlMode.Position, encoderCounts, DemandType.ArbitraryFeedForward, arbitraryFF);
    }

    public void updateSmartDashboard() {
        Shuffleboard.putNumber("Wrist","Encoder Count", getPosition());
        Shuffleboard.putNumber("Wrist","Encoder Velocity", getVelocity());
        Shuffleboard.putBoolean("Wrist","Encoder Health", isEncoderHealthy());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new UpdateWristSetpoint());
    }
}
