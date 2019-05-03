/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PCM_ONE;

public class Harpoon extends Subsystem {

    DoubleSolenoid harpoon = new DoubleSolenoid(PCM_ONE.ADDRESS, PCM_ONE.HATCH_EXTEND.FORWARD, PCM_ONE.HATCH_EXTEND.REVERSE);

    public Harpoon() {
        super("harpoon");

    }

    public boolean getHarpoonExtendStatus(){
        return harpoon.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setHarpoonExtend(boolean state){
        harpoon.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void updateShuffleboard() {
//        Shuffleboard.putNumber("Intake","Intake State", intakeState);
    }

    public void updateSmartDashboard() {
//        SmartDashboard.putBoolean("Cargo", intakeIndicator[2]);
//        SmartDashboard.putBoolean("Hatch Ground", intakeIndicator[1]);
//        SmartDashboard.putBoolean("Hatch", intakeIndicator[0]);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
