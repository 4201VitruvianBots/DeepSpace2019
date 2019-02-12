package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.RobotMap;
import frc.vitruvianlib.driverstation.Shuffleboard;

import javax.naming.ldap.Control;

public class Controls {

    public static PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.pdp);

    public Controls() {

    }

    public void initTestSettings() {
        Shuffleboard.putNumber("Elevator", "Test Voltage", 0);
    }

    public static double getElevatorLeftCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorLeft);
    }

    public static double getElevatorRightCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorRight);
    }
}
