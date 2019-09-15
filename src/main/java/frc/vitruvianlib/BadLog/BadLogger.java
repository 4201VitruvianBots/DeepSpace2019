package frc.vitruvianlib.BadLog;

import badlog.lib.BadLog;
import badlog.lib.DataInferMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import java.io.File;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.Optional;

public class BadLogger {
	private static BadLog logger;
	static boolean isRunning = false;
	static boolean isMatch = false;
	static String usbPath = "/media/sda1/4201Robot/Logs/";
    static String roboRioPath = "/home/lvuser/4201Robot/Logs/";
    static String  basePath, logPath, logName;
    public static double m_logStartTime;
    private static long lastLogTime;

    public static SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

	/* This is a wrapper class for team 1014's BadLog logging framework. This is so that we can use their
	 * logging library, while maintaining the coding structure of our previous logging framework.
	 */
    public static BadLog getInstance() {
        if (logger == null)
        	startLogger();
        return logger;
    }
    
    public static void startLogger() {
        // Check if base path is a valid directory
        try {
            File baseDir = new File(usbPath);
            if(baseDir.isDirectory() && baseDir.exists() && baseDir.canWrite()) {
                System.out.println("VitruvianLogger Info: Base Path found! Logging to USB...");
                basePath = usbPath;
            } else
                basePath = roboRioPath;
        } catch(Exception e) {
            System.out.println("VitruvianLogger Error: USB Path not detected. Backing up to roboRIO path.");
            basePath = roboRioPath;
        }

        // Stop logging if its already running to avoid issues and to write logs to new directory
        if (isRunning && !isMatch)
            stopLogger();

        // Determine where to save logs
        DriverStation.MatchType matchType = DriverStation.getInstance().getMatchType();
        m_logStartTime = Timer.getFPGATimestamp();
        logPath = basePath;

        if (matchType == DriverStation.MatchType.None) {
            if (DriverStation.getInstance().isDisabled()) {
                logPath += "Disabled/";
            } else if (DriverStation.getInstance().isAutonomous()) {
                logPath += "Auto/";
            } else if (DriverStation.getInstance().isOperatorControl()) {
                logPath += "TeleOp/";
            } else {
                logPath += "UnsortedLog/";
            }

            // Use timestamp for log location
            Timestamp timestamp = new Timestamp(System.currentTimeMillis());
            logName = dateFormat.format(timestamp) + ".bag";
        } else {
        	// TODO: Debug this section - match logs aren't generated 
            isMatch = true;
            String eventString = DriverStation.getInstance().getEventName();
            logPath += eventString + "/";
            String matchNo = String.format("%02d", DriverStation.getInstance().getMatchNumber());

            if (matchType == DriverStation.MatchType.Practice) {
                logPath += "Practice/";
                logName = "P" + matchNo + ".bag";
            } else if (matchType == DriverStation.MatchType.Qualification) {
                logPath += "Qualification/";
                logName = "QM" + matchNo + ".bag";
            } else if (matchType == DriverStation.MatchType.Elimination) {
            	// TODO: How to determine elim match type (QF, SF, F)?
                logPath += "Elimination/";
                logName = "E" + matchNo + ".bag";
            } else {
                logPath += "UnknownMatch/" + matchNo + "/";
                logName = "UN" + matchNo + ".bag";
            }
        }
        File dirPath = new File(logPath);
        if (!dirPath.exists() && !dirPath.isDirectory())
            dirPath.mkdirs();
        try {
            logger = BadLog.init(logPath + logName);
            {
                BadLog.createValue("Start Time", dateFormat.format(m_logStartTime));
                BadLog.createValue("Event Name",
                        Optional.ofNullable(DriverStation.getInstance().getEventName()).orElse(""));
                BadLog.createValue("Match Type", DriverStation.getInstance().getMatchType().toString());
                BadLog.createValue("Match Number", "" + DriverStation.getInstance().getMatchNumber());
                BadLog.createValue("Alliance", DriverStation.getInstance().getAlliance().toString());
                BadLog.createValue("Location", "" + DriverStation.getInstance().getLocation());

                BadLog.createTopicSubscriber("Time", "s", DataInferMode.DEFAULT, "hide", "delta", "xaxis");

                BadLog.createTopicStr("System/Browned Out", "bool", () -> fromBool(RobotController.isBrownedOut()));
                BadLog.createTopic("System/Battery Voltage", "V", () -> RobotController.getBatteryVoltage());
                BadLog.createTopicStr("System/FPGA Active", "bool", () -> fromBool(RobotController.isSysActive()));
                BadLog.createTopic("Match Time", "s", () -> DriverStation.getInstance().getMatchTime());

                Robot.driveTrain.initLogging();
                Robot.elevator.initLogging();
            }
            logger.finishInitialization();
        } catch (Exception e){
        }
        isRunning = true;
    }

    public static void stopLogger() {
        isRunning = false;
        isMatch = false;
    }
    
    public static void updateLogs() {
    	if(isRunning) {
            double currentTime = ((Timer.getFPGATimestamp() - m_logStartTime)) / 1_000_000_000d;
            BadLog.publish("Time", currentTime);

            long currentTimeMS = System.currentTimeMillis();
//            if (!DriverStation.getInstance().isDisabled() || currentTimeMS - lastLogTime >= 250) {
            if (currentTimeMS - lastLogTime >= 400) {
                lastLogTime = currentTimeMS;
                logger.updateTopics();
                logger.log();
            }
    	}
    }

    public static String fromBool(boolean in) {
        // Saves on disk usage by not including decimals
        return in ? "1" : "0";
    }
}
