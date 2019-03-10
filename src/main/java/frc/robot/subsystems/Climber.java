/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public TalonSRX master = new TalonSRX(RobotMap.climbMotor);

    DoubleSolenoid climbPistons = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.climbPistonForward, RobotMap.climbPistonReverse);

    public Climber() {
        super("Climber");
    }

    public void setClimberOutput(double output){
        master.set(ControlMode.PercentOutput, output);
    }

    public void setClimbPistonState(boolean state) {
        climbPistons.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public boolean getClimbPistonState() {
        return climbPistons.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public boolean isClimbMode() {
        return Elevator.controlMode == 0 && Wrist.controlMode == 0;
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Climb Mode", isClimbMode());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
