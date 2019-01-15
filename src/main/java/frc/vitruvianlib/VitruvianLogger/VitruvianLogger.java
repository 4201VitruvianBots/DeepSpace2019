package frc.vitruvianlib.VitruvianLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

public class VitruvianLogger {

    static VitruvianLogger logger;
    boolean isRunning = false;
    boolean isMatch = false;
    String basePath = "/media/sda1/4201Robot/Logs/";
    String logPath;
    List<VitruvianLog> logList = new ArrayList<>();
    public static double m_logStartTime;

    public static SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

    private VitruvianLogger() {

    }

    public static VitruvianLogger getInstance() {
        if (logger == null)
            logger = new VitruvianLogger();
        return logger;
    }

    public void addLog(VitruvianLog log) {
        logList.add(log);
    }

    public void removeLog(VitruvianLog log) {
        // TODO: Check implementation
        logList.remove(log);
    }

    public void startLogger() {
        // Stop logging if its already running to avoid issues and to write logs to new directory
        if (isRunning && !isMatch)
            stopLogger();

        // Determine where to save logs
        DriverStation.MatchType matchType = DriverStation.getInstance().getMatchType();
        String eventString = DriverStation.getInstance().getEventName();
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
            logPath += dateFormat.format(timestamp) + "/";
        } else {
            isMatch = true;
            logPath += eventString + "/";
            String matchNo = String.format("%02d", DriverStation.getInstance().getMatchNumber());

            if (matchType == DriverStation.MatchType.Practice)
                logPath += "Practice/" + matchNo + "/";
            else if (matchType == DriverStation.MatchType.Qualification)
                logPath += "Qualification/" + matchNo + "/";
            else if (matchType == DriverStation.MatchType.Elimination)
                logPath += "Elimination/" + matchNo + "/";
            else
                logPath += "UnknownMatch/" + matchNo + "/";
        }

        // Contingency if multiple matches exist (i.e. a match replay)
        File folderPath = new File(logPath);
        int counter = 1;
        while (folderPath.exists() && folderPath.isDirectory() && counter < 99) {
            String newPath = logPath + "_" + String.format("%02d", counter++);
            folderPath = new File(newPath);
        }
        if (counter == 99)
            System.out.println("VitruvianLogger Error: Counter has reached 99. " +
                    "Will start overwriting logs");

        // Make new Logger directories
        boolean startLogger = true;
        try {
            folderPath.mkdirs();
        } catch (Exception e) {
            System.out.println("VitruvianLogger Error: Could not make log dirs. " +
                    "Defaulting to known path");
            e.printStackTrace();

            counter = 1;
            logPath = "/media/sda1/4201Robot/Logs/UnsortedLogs/";
            while (folderPath.exists() && folderPath.isDirectory() && counter < 99) {
                String newPath = logPath + "_" + String.format("%02d", counter++);
                folderPath = new File(newPath);
            }
            try {
                folderPath.mkdirs();
            } catch (Exception ex) {
                System.out.println("VitruvianLogger Error: Could not default to backup folder. " +
                        "Logger will not start");
                ex.printStackTrace();
                startLogger = false;
            }
        }

        // Start all loggers
        if (startLogger)
            for (VitruvianLog log : logList)
                log.startLogging(logPath);

        isRunning = true;
    }

    public void stopLogger() {
        for (VitruvianLog log : logList)
            log.stopLogging();

        isRunning = false;
        isMatch = false;
    }
}
