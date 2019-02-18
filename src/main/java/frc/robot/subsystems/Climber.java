package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Climber extends Subsystem {

    TalonSRX cilmbMotor = new TalonSRX(RobotMap.climbMotor);

    public Climber() {
        super("Climber");

        cilmbMotor.configFactoryDefault();
        cilmbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setClimbMotorVoltage(double voltage) {
        cilmbMotor.set(ControlMode.PercentOutput, voltage/12);
    }

    @Override
    protected void initDefaultCommand() {

    }
}
