/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.vitruvianlib.drivers.FactoryTalonSRX;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX[] driveMotors = {
    FactoryTalonSRX.createDefaultTalon(RobotMap.leftFrontDriveMotor),
    FactoryTalonSRX.createPermanentSlaveTalon(RobotMap.leftRearDriveMotor, RobotMap.leftFrontDriveMotor),
    FactoryTalonSRX.createDefaultTalon(RobotMap.rightFrontDriveMotor),
    FactoryTalonSRX.createPermanentSlaveTalon(RobotMap.rightRearDriveMotor, RobotMap.rightFrontDriveMotor)
  };

  DoubleSolenoid driveTrainShifters = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.driveTrainShifterForward, RobotMap.driveTrainShifterReverse);
  public AHRS navX = new AHRS(SPI.Port.kMXP);

  public DriveTrain(){
    super("DriveTrain");

    driveMotors[0].setInverted(false);
    driveMotors[1].setInverted(false);
    driveMotors[2].setInverted(true);
    driveMotors[3].setInverted(true);
  }

  public int getLeftEncoderCount(){
    return driveMotors[0].getSelectedSensorPosition();
  }

  public int getRightEncoderCount(){
    return driveMotors[2].getSelectedSensorPosition();
  }

  public void setMotorArcadeDrive(double throttle, double turn) {
    double leftOutput = throttle + turn;
    double rightOutput = throttle - turn;

    setMotorPercentOutput(leftOutput, rightOutput);
  }

  public void setMotorTankDrive(double leftOutput, double rightOutput) {

    setMotorPercentOutput(leftOutput, rightOutput);
  }

  public void setMotorVelocityOutput(double leftOutput, double rightOutput) {
    double k_maxVelocity = 6;
    double leftVelocity = leftOutput * k_maxVelocity;
    double rightVelocity = rightOutput * k_maxVelocity;

    if(Math.abs(leftVelocity) > k_maxVelocity)
      leftVelocity = (leftVelocity > k_maxVelocity) ? k_maxVelocity : k_maxVelocity;

    if(Math.abs(rightVelocity) > k_maxVelocity)
        rightVelocity = (rightVelocity > k_maxVelocity) ? k_maxVelocity : k_maxVelocity;

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
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("NavX Temp (C)", navX.getTempC());
    SmartDashboard.putNumber("Angle", navX.getAngle());
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
