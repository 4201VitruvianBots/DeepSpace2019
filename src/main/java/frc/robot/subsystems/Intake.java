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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.INTAKE_SPEED;
import frc.vitruvianlib.driverstation.Shuffleboard;

public class Intake extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public static int intakeState = 0;
    public static int outtakeState = 0;

    public boolean[] intakeIndicator = {false, false, false};

    public static boolean overridePassive = false;
    public static boolean enableBannerSensor = true;
    private boolean isBannerTripped = false;

//    DoubleSolenoid harpoonExtend = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeExtendForward, RobotMap.hatchIntakeExtendReverse);
    //DoubleSolenoid harpoonSecure = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.hatchIntakeSecureForward, RobotMap.hatchIntakeSecureReverse);

    private TalonSRX[] intakeMotors = {
        new TalonSRX(RobotMap.cargoIntakeMotor),
        new TalonSRX(RobotMap.hatchIntakeMotor)
    };

    public DigitalInput bannerIR = new DigitalInput(RobotMap.bannerIR);
    // TODO: Add sensor for hatch intake

    public Intake() {
        super("Intake");

        for(TalonSRX intakeMotor:intakeMotors) {
            intakeMotor.configFactoryDefault();
            intakeMotor.setNeutralMode(NeutralMode.Coast);
            intakeMotor.configContinuousCurrentLimit(30);
//            intakeMotor.configPeakCurrentLimit(40);
//            intakeMotor.configPeakCurrentDuration(2000);
            intakeMotor.enableCurrentLimit(true);
        }
        intakeMotors[0].setInverted(false);
        intakeMotors[1].setInverted(true);
        //intakeMotors[1].set(ControlMode.Follower, intakeMotors[0].getDeviceID());
    }

    public void setCargoIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setHatchGroundIntakeOutput(double output){
        intakeMotors[0].set(ControlMode.PercentOutput, -output);
    }

//    public boolean getHarpoonSecureStatus(){
//        return harpoonSecure.get() == DoubleSolenoid.Value.kForward ? true : false;
//    }

//    public boolean getHarpoonExtendStatus(){
//        return harpoon.get() == DoubleSolenoid.Value.kForward ? true : false;
//    }

//    public void setHarpoonExtend(boolean state){
//        if (state)
//            harpoon.set(DoubleSolenoid.Value.kForward);
//        else
//            harpoon.set(DoubleSolenoid.Value.kReverse);
//    }

//    public void setHarpoonSecure(boolean state){
//        if (state)
//            harpoonSecure.set(DoubleSolenoid.Value.kForward);
//        else
//            harpoonSecure.set(DoubleSolenoid.Value.kReverse);
//    }

    public void setHatchIntakeOutput(double output){
        intakeMotors[1].set(ControlMode.PercentOutput, output);
    }

    public void updateIntakeIndicator() {
        for(int i = 0; i < intakeIndicator.length; i++)
            intakeIndicator[i] = false;
        intakeIndicator[intakeState] = true;
    }

    public void updateCargoIntakeState() {
        if(enableBannerSensor) {
            if (Robot.m_oi.rightButtons[0].get()) {

            } else if (bannerIR.get() && !isBannerTripped) {
//            Timer.delay(0.5);
                setCargoIntakeOutput(INTAKE_SPEED.CARGO_HOLD);
//            isTripped = true;
            } else if (isBannerTripped) {
                setCargoIntakeOutput(0);
                isBannerTripped = false;
            }
        }
    }
    public void updateOuttakeState() {
//        if(bannerIR.get())
//            outtakeState = 2;
//        else if(false)  // TODO: Add hatch sensor
//            outtakeState = 1;
//        else
            outtakeState = intakeState;
    }

    public void updateShuffleboard() {
        Shuffleboard.putNumber("Intake","Intake State", intakeState);
        Shuffleboard.putBoolean("Intake","Banner IR", bannerIR.get());

        Shuffleboard.putBoolean("Controls","Cargo", intakeIndicator[2]);
        Shuffleboard.putBoolean("Controls","Hatch", intakeIndicator[0]);
        Shuffleboard.putBoolean("Controls","Banner Enabled", enableBannerSensor);
//        Shuffleboard.putBoolean("Controls","Banner IR", bannerIR.get());
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Cargo", intakeIndicator[2]);
//        SmartDashboard.putBoolean("Hatch Ground", intakeIndicator[1]);
        SmartDashboard.putBoolean("Hatch", intakeIndicator[0]);
        SmartDashboard.putBoolean("Banner IR", bannerIR.get());
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
//        setDefaultCommand(new ConditionalCommand(new IntakePassive()) {
//            @Override
//            protected boolean condition() {
//                return !overridePassive;
//            }
//        });
    }
}
