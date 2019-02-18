package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.test.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.vitruvianlib.driverstation.Shuffleboard;
import org.ini4j.Wini;

import java.io.File;

public class Controls {

    String roboRIOFilepath = "/home/lvuser/4201Robot/";
    String iniFilename = "DeepSpace2019.ini";

    public static PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.pdp);

    public DigitalInput whichRobot = new DigitalInput(RobotMap.robotSwitch);

    public Controls() {

    }

    public void readIniFile() {
        try {
            File iniFile = new File(roboRIOFilepath + iniFilename);
            if(!iniFile.exists()) {
                File folderPath = new File(roboRIOFilepath);
                folderPath.mkdirs();
                iniFile.createNewFile();

                Wini robotIni = new Wini(iniFile);
                robotIni.put("Elevator", "Encoder_Calibration", 0);
                robotIni.put("Wrist", "Encoder_Calibration", 0);
                robotIni.store(iniFile);
            } else {
                Wini robotIni = new Wini(iniFile);
                Elevator.calibrationValue  = Integer.valueOf(robotIni.get("Elevator", "Encoder_Calibration"));
                Wrist.calibrationValue = Integer.valueOf(robotIni.get("Wrist", "Encoder_Calibration"));
                Shuffleboard.putNumber("Controls", "Initial Elevator Calibration", Elevator.calibrationValue);
                Shuffleboard.putNumber("Controls", "Initial Wrist Calibration", Wrist.calibrationValue);
            }
        } catch (Exception e) {

        }
    }

    public void writeIniFile(String section, String key, String value) {
        try {
            File iniFile = new File("/home/lvuser/4201Robot/DeepSpace2019.ini");
            Wini robotIni = new Wini(iniFile);
            robotIni.put(section, key, value);
            robotIni.store(iniFile);
        } catch (Exception e) {

        }
    }

    public void initTestSettings() {
        Shuffleboard.putNumber("Elevator", "Test Voltage", 0);
        Shuffleboard.putData("Controls", new ToggleElevatorControlMode());
        Shuffleboard.putData("Controls", new ToggleWristControlMode());
        Shuffleboard.putData("Controls", new ZeroElevatorEncoder());
        Shuffleboard.putData("Controls", new ZeroWristEncoder());
    }

    public static double getElevatorLeftCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorLeft);
    }

    public static double getElevatorRightCurrent() {
        return pdp.getCurrent(RobotMap.pdpChannelElevatorRight);
    }
}
