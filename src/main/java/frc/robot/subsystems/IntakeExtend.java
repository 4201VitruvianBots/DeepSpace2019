/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.vitruvianlib.driverstation.Shuffleboard;

public class IntakeExtend extends Subsystem {

    DoubleSolenoid harpoonExtend = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeExtendForward, RobotMap.hatchIntakeExtendReverse);

    public IntakeExtend() {
        super("IntakeExtend");

    }

    public boolean getHarpoonExtendStatus(){
        return harpoonExtend.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setHarpoonExtend(boolean state){
        harpoonExtend.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void updateShuffleboard() {
//        Shuffleboard.putNumber("Intake","Intake State", intakeState);
//        Shuffleboard.putBoolean("Intake","Banner IR", bannerIR.get());
    }

    public void updateSmartDashboard() {
//        SmartDashboard.putBoolean("Cargo", intakeIndicator[2]);
//        SmartDashboard.putBoolean("Hatch Ground", intakeIndicator[1]);
//        SmartDashboard.putBoolean("Hatch", intakeIndicator[0]);
//        SmartDashboard.putBoolean("Banner IR", bannerIR.get());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
