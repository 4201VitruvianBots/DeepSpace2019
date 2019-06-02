/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PCM_ONE;
import frc.vitruvianlib.drivers.CachedDoubleSolenoid;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climber extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    CachedDoubleSolenoid climbPistons = new CachedDoubleSolenoid(PCM_ONE.CAN_ADDRESS, PCM_ONE.CLIMB_PISTONS.FORWARD, PCM_ONE.CLIMB_PISTONS.REVERSE);

    public int climbMode = 0;
    public static boolean initClimb = false;

    public Climber() {
        super("Climber");
    }

    public void setClimbPistonState(boolean state) {
        climbPistons.set(state ? Value.kForward : Value.kReverse);
    }

    public boolean getClimbPistonState() {
        return climbPistons.get() == Value.kForward;
    }

    public void updateShuffleboard() {
        Shuffleboard.putBoolean("Controls", "Climb Mode", climbMode == 1);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
