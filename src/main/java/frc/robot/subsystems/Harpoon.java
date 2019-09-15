/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PCM_ONE;
import frc.vitruvianlib.drivers.CachedDoubleSolenoid;

public class Harpoon extends Subsystem {

    CachedDoubleSolenoid harpoon = new CachedDoubleSolenoid(PCM_ONE.CAN_ADDRESS, PCM_ONE.HATCH_EXTEND.FORWARD, PCM_ONE.HATCH_EXTEND.REVERSE);

    public Harpoon() {
        super("harpoon");

    }

    public boolean getHarpoonExtendStatus(){
        return harpoon.get() == Value.kForward ? true : false;
    }

    public void setHarpoonExtend(boolean state){
        harpoon.set(state ? Value.kForward : Value.kReverse);
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
