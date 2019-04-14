package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SaveMe;
import frc.robot.commands.climber.DisableClimbSequence;
import frc.robot.commands.climber.EnableClimbSequence;
import frc.robot.commands.climber.SetLimelightLEDMode;
import frc.robot.commands.test.*;
import frc.vitruvianlib.driverstation.Shuffleboard;
import org.ini4j.Wini;

import java.io.File;

public class Controls extends Subsystem {

    public static PowerDistributionPanel pdp = new PowerDistributionPanel(RobotMap.pdp);

    Compressor compressor = new Compressor();

    public DigitalInput whichRobot = new DigitalInput(RobotMap.robotSwitch);

    public Controls() {
        super("Controls");
    }

    public void readIniFile() {
        try {

            File iniFile = new File("/home/lvuser/4201Robot/DeepSpace2019.ini");
            if(!iniFile.exists()) {
                iniFile.mkdirs();
                iniFile.createNewFile();

                Wini robotIni = new Wini(iniFile);
                robotIni.put("Elevator", "Encoder_Calibration", 0);
                robotIni.put("Wrist", "Encoder_Calibration", 0);
                robotIni.store(iniFile);
            } else {
                Wini robotIni = new Wini(iniFile);
                int elevatorCalibration = Integer.getInteger(robotIni.get("Elevator", "Encoder_Calibration"));
                int wristCalibration = Integer.getInteger(robotIni.get("Wrist", "Encoder_Calibration"));
                Shuffleboard.putNumber("Controls", "Initial Elevator Calibration", elevatorCalibration);
                Shuffleboard.putNumber("Controls", "Initial Wrist Calibration", elevatorCalibration);
//                Elevator.calibrationValue = Integer.getInteger(robotIni.get("Elevator", "Encoder_Calibration"));
//                Wrist.calibrationValue = Integer.getInteger(robotIni.get("Wrist", "Encoder_Calibration"));
//                Robot.elevator.setEncoderPosition(Elevator.calibrationValue);
//                Robot.wrist.setEncoderPosition(Wrist.calibrationValue);
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
//        Shuffleboard.putNumber("Elevator", "Test Voltage", 0);
        Shuffleboard.putData("Controls", new ToggleDriveTrainControlMode());
        Shuffleboard.putData("Controls", new ToggleElevatorControlMode());
        Shuffleboard.putData("Controls", new ToggleWristControlMode());
        Shuffleboard.putData("Controls", new ZeroElevatorEncoder());
        Shuffleboard.putData("Controls", new ZeroWristEncoder());
//        Shuffleboard.putData("Controls", new SaveMe());
        Shuffleboard.putData("Controls", new EnableClimbSequence());
        Shuffleboard.putData("Controls", new DisableClimbSequence());

        Shuffleboard.putData("Controls", new SetLimelightLEDMode(0));

//        Shuffleboard.putData("Controls", new VictorySpin(10));
    }

    public static double getPdpCurrent(int pdpChannel) {
        return pdp.getCurrent(pdpChannel);
    }

    public void setCompressorState(boolean enable) {
        if(enable)
            compressor.start();
        else
            compressor.stop();
    }
    @Override
    protected void initDefaultCommand() {

    }
}
