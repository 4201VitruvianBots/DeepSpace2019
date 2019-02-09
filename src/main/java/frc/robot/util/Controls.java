package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.RobotMap;

import javax.naming.ldap.Control;

public class Controls {

    public static PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.pdp);

    public Controls() {

    }

    public static double getElevatorLeftCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorLeft);
    }

    public static double getElevatorRightCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorRight);
    }
}
