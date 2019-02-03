/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.Controls;
import frc.vitruvianlib.VitruvianLogger.VitruvianLog;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;

/**
 * Add your docs here.
 */
public class NerdyElevator extends Subsystem {

  private TalonSRX[] elevatorMotors = {
          new TalonSRX(RobotMap.leftElevator),
          new TalonSRX(RobotMap.rightElevator),
  };

  DigitalInput[] limitSwitches = {
          new DigitalInput(RobotMap.elevatorBottom),
          new DigitalInput(RobotMap.elevatorTop)
  };

  public int upperLimitEncoderCounts = 0; //position of upper limit in encoder counts
  public int lowerLimitEncoderCounts = 0; //position of lower limit in encoder counts

  private double m_distanceRatio, m_distanceOffset;

  // This is in STU (Stupid Talon Units, AKA ticks/decisecond)
  protected static final double kStaticFrictionDeadband = 5;
  // These are initialized in volts using the config methods, but are stored as percent outputs
  protected double m_gravityFF, m_staticFF;

  public NerdyElevator() {
    super("elevator");
    m_distanceRatio = 1;
    m_distanceOffset = 0;
    for (TalonSRX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setInverted(true);
      motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }
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

  public void configFFs(double newGravityFF, double newStaticFF) {
    this.configGravityFF(newGravityFF);
    this.configStaticFF(newStaticFF);
  }

  public void configGravityFF(double newGravityFF) {
    this.m_gravityFF = newGravityFF / 12.0;
  }

  public void configStaticFF(double newStaticFF) {
    this.m_staticFF = newStaticFF / 12.0;
  }

  public void configDistanceRatio(double newDistanceRatio) {
    m_distanceRatio = newDistanceRatio;
  }

  public void configDistanceOffset(double newDistanceOffset) {
    m_distanceOffset = newDistanceOffset;
  }

  public void configHeightConversion(double newDistanceRatio, double newDistanceOffset) {
    m_distanceRatio = newDistanceRatio;
    m_distanceOffset = newDistanceOffset;
  }

  public boolean getUpperLimitSensor(){
    return limitSwitches[0].get();
  }

  public boolean getLowerLimitSensor(){
    return limitSwitches[1].get();
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

  public boolean getLeftElevatorEncoderHealth() {
    return elevatorMotors[0].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
  }

  public boolean getRightElevatorEncoderHealth() {
    return elevatorMotors[1].getSensorCollection().getPulseWidthRiseToFallUs() != 0;
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

  public boolean isNotMoving() {
    return Math.abs(getVelocityEncoderCounts()) <= kStaticFrictionDeadband;
  }

  public double getFFIfMoving() {
    return m_gravityFF;
  }

  public double getFFIfNotMoving(double error) {
    double sign = Math.signum(error);
    return m_gravityFF + sign * m_staticFF;
  }

  //Not going to be used
  public void setPowerWithFF(double power) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(power));
    } else {
      elevatorMotors[0].set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setVoltageWithFF(double voltage) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.PercentOutput, voltage/12.0, DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(voltage));
    } else {
      elevatorMotors[0].set(ControlMode.PercentOutput, voltage/12.0, DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setPosition(double pos) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.Position, pos, DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(pos - getPositionEncoderCounts()));
    } else {
      elevatorMotors[0].set(ControlMode.Position, pos, DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setHeight(double height) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.Position, heightToTicks(height), DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(height - getHeight()));
    } else {
      elevatorMotors[0].set(ControlMode.Position, heightToTicks(height), DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setPositionMotionMagic(double pos) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(pos - getPositionEncoderCounts()));
    } else {
      elevatorMotors[0].set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setHeightMotionMagic(double height) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.MotionMagic, heightToTicks(height), DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(height - getHeight()));
    } else {
      elevatorMotors[0].set(ControlMode.MotionMagic, heightToTicks(height), DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public void setVelocity(double vel) {
    if (isNotMoving()) {
      elevatorMotors[0].set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward,
        getFFIfNotMoving(vel - getVelocityEncoderCounts()));
    } else {
      elevatorMotors[0].set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, getFFIfMoving());
    }
  }

  public double heightToTicks(double height) {
    return (height - m_distanceOffset) / m_distanceRatio;
  }

  public double ticksToHeight(double ticks) {
    return (m_distanceRatio * ticks) + m_distanceOffset;
  }

  public double getHeight() {
    return ticksToHeight(this.getPositionEncoderCounts());
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator Height", getHeight());
  }


  @Override
  protected void initDefaultCommand() {

  }
}