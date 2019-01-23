/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Intake extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    DoubleSolenoid harpoon = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeForward, RobotMap.hatchIntakeReverse);

    private TalonSRX[] intakeMotors = {
        new TalonSRX(RobotMap.hatchIntakeForward),
        new TalonSRX(RobotMap.hatchIntakeReverse)
    };

    public Intake() {
        super("Intake");
        for (int i = 0; i < intakeMotors.length; i++) {
            intakeMotors[i].configFactoryDefault();
        }
    }

    public void setIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public boolean getHarpoonStatus(){
        return harpoon.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setHarpoonForward(){
        harpoon.set(DoubleSolenoid.Value.kForward);
    }

    public void setHarpoonReverse(){
        harpoon.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
