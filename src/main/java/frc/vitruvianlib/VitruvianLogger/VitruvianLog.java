package frc.vitruvianlib.VitruvianLogger;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

public class VitruvianLog implements Runnable {

    private String logName;
    private List<VitruvianLogField> logFields = new ArrayList<>();
    private double logPeriod;
    Notifier notifier = new Notifier(this);

    File logFile;
    private FileWriter writeToLog;

    public VitruvianLog(String logName, double logPeriod) {
        this.logName = logName;
        this.logPeriod = logPeriod;

        addLogField("Timestamp", this::getTimestamp);
    }

    public void addLogField(String fieldName, VitruvianLogField.FunctionCall function) {
        logFields.add(new VitruvianLogField(fieldName, function));
    }

    public void removeLogField(String fieldName) {
        // TODO: Check logic/implementation. Java List remove object with a name
        logFields.remove(fieldName);
    }

    public String getTimestamp() {
        return String.format("%.2f", Timer.getFPGATimestamp() - VitruvianLogger.m_logStartTime);
    }

    public void startLogging(String logPath) {
        try {
            // Initialize file writer
            logFile = new File(logPath + logName + ".csv");
            writeToLog = new FileWriter(logFile);

            // Write a header to define column names
            StringBuilder header = new StringBuilder();
            for (VitruvianLogField value : logFields) {
                header.append(value.name + ", ");
            }
            // Remove the last comma and space
            header.delete(header.length() - 2, header.length());

            writeToLog.append(header + "\n");

            notifier.startPeriodic(logPeriod);
        } catch (Exception e) {
            System.out.println("VitruvianLog Error: " + logName + " Couldn't write to file");
            e.printStackTrace();
            stopLogging();
        }
    }

    public void stopLogging() {
        notifier.stop();
        try {
            writeToLog.close();
        } catch (Exception e) {
            System.out.println("VitruvianLog Error: " + logName + " Couldn't close log file writer");
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        try {
            StringBuilder resultsToWrite = new StringBuilder();

            // Populate a row with data.
            for (VitruvianLogField value : logFields) {
                resultsToWrite.append(value.functionHandle.get().toString() + ", ");
            }
            // Remove the last comma and space
            resultsToWrite.delete(resultsToWrite.length() - 2, resultsToWrite.length());

            writeToLog.append(resultsToWrite + "\n");
        } catch (Exception e) {
            System.out.println("VitruvianLog Error: " + logName + " Couldn't write to file");
            e.printStackTrace();
            stopLogging();
        }
    }
}
