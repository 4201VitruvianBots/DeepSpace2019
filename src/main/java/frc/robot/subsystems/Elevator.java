package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

    public TalonSRX[] elevatorMotors= {
        new TalonSRX(30),
        new TalonSRX(31)
    };

    public Elevator() {
        super("Elevator");

        for(TalonSRX talonSRX:elevatorMotors)
            talonSRX.configFactoryDefault();

        elevatorMotors[0].setInverted(true);
        elevatorMotors[1].setInverted(true);
        elevatorMotors[1].set(ControlMode.Follower, elevatorMotors[0].getDeviceID());
    }

    @Override
    protected void initDefaultCommand() {

    }
}
