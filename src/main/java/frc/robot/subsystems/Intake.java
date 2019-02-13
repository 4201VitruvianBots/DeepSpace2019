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

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Intake extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public static int intakeState = 0;
    public static int outtakeState = 0;

    DoubleSolenoid harpoonExtend = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeExtendForward, RobotMap.hatchIntakeExtendReverse);
    DoubleSolenoid harpoonSecure = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeSecureForward, RobotMap.hatchIntakeSecureReverse);

    private TalonSRX[] intakeMotors = {
        new TalonSRX(RobotMap.hatchIntakeMotor),
        //new TalonSRX(RobotMap.hatchIntakeMotor)
    };

    public DigitalInput bannerIR = new DigitalInput(RobotMap.bannerIR);
    // TODO: Add sensor for hatch intake

    public Intake() {
        super("Intake");

        for(TalonSRX intakeMotor:intakeMotors) {
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
        }
        intakeMotors[0].setInverted(true);
        //intakeMotors[1].setInverted(true);
        //intakeMotors[1].setInverted(true);
        //intakeMotors[1].set(ControlMode.Follower, intakeMotors[0].getDeviceID());
    }

    public void setCargoIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, output);
        //intakeMotors[1].set(ControlMode.PercentOutput, -output);
    }

    public void setHatchGroundIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, -output);
        //intakeMotors[1].set(ControlMode.PercentOutput, -output);
    }

    public boolean getHarpoonSecureStatus(){
        return harpoonSecure.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public boolean getHarpoonExtendStatus(){
        return harpoonExtend.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setHarpoonExtend(boolean state){
        if (state)
            harpoonExtend.set(DoubleSolenoid.Value.kForward);
        else
            harpoonExtend.set(DoubleSolenoid.Value.kReverse);
    }

    public void setHarpoonSecure(boolean state){
        if (state)
            harpoonSecure.set(DoubleSolenoid.Value.kForward);
        else
            harpoonSecure.set(DoubleSolenoid.Value.kReverse);
    }

    public void updateOuttakeState() {
        if(!bannerIR.get())
            outtakeState = 2;
        else if(false)  // TODO: Add hatch sensor
            outtakeState = 1;
        else
            outtakeState = intakeState;
    }

    public void updateSmartDashboard() {
        Shuffleboard.putNumber("Intake","Intake State", intakeState);
        Shuffleboard.putBoolean("Intake","Banner IR", bannerIR.get());

        SmartDashboard.putNumber("Intake State", intakeState);
        SmartDashboard.putBoolean("Banner IR", bannerIR.get());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
