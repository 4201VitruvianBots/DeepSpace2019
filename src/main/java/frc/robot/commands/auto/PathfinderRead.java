/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.vitruvianlib.driverstation.Shuffleboard;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

/**
 * An example command.  You can replace me with your own command.
 */
public class PathfinderRead extends Command {
    String filename;
    Trajectory leftTrajectory;
    Trajectory rightTrajectory;
    EncoderFollower left;
    EncoderFollower right;
    boolean end = false;
    Timer stopwatch;
    Waypoint[] points;
    boolean first = false;
    Notifier periodicRunnable;

    double kP = 0.8;
    double kD = 0.002;
    double max_vel = 1.6035;

    public PathfinderRead(String filename) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        this.filename = filename;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Used to print status to dashboard for debugging
        Shuffleboard.putString("Pathfinder", "PathFinder Status", "Initializing...");

        try {
            // Try to read the .csv files
            File leftFile = new File(Filesystem.getDeployDirectory() + "/output/" + filename + ".left.pf1.csv");
            Trajectory lT = Pathfinder.readFromCSV(leftFile);
            leftTrajectory = lT;

            File rightFile = new File(Filesystem.getDeployDirectory() + "/output/" + filename + ".right.pf1.csv");
            Trajectory rT = Pathfinder.readFromCSV(rightFile);
            rightTrajectory = rT;

            this.setTimeout(lT.segments.length * 0.05 * 1.1);
        } catch (Exception e) {
            // Handle it how you want
            /*
            if(first){
                    File leftFile = new File("/media/sda1/Pathfinder/straightCalibration_Left.csv");
                    Trajectory lT = Pathfinder.readFromCSV(leftFile);
                    leftTrajectory = lT;
                    File rightFile = new File("/media/sda1/Pathfinder/straightCalibration_Right.csv");
                    Trajectory rT = Pathfinder.readFromCSV(rightFile);
                    rightTrajectory =  rT;

                    Shuffleboard.putString("Pathfinder", "PathFinder Read" , "Trajectory Read Failed!");
            }
            */
        }

        // This resets the trajectory counts so you can run autos successively without redeploying the robot code.
        // This is in a try/catch statement because the first path loaded will always be null
        try {
            left.reset();
            right.reset();
        } catch (Exception e) {
        }

        // Configure encoder classes to follow the trajectories
        left = new EncoderFollower(leftTrajectory);
        right = new EncoderFollower(rightTrajectory);

        Shuffleboard.putString("Pathfinder", "PathFinder Status", "Enabling...");

        left.configureEncoder(Robot.driveTrain.getLeftEncoderCount(), 4096, RobotMap.wheel_diameter);
        right.configureEncoder(Robot.driveTrain.getLeftEncoderCount(), 4096, RobotMap.wheel_diameter);

        // The A value here != max_accel. A here is an acceleration gain (adjusting acceleration to go faster/slower), while max_accel is the max acceleration of the robot.
        // Leave A here alone until robot is reaching its target, then adjust to get it to go faster/slower (typically a small value like ~0.03 is used).
        // Usually, you wont have to adjust this though.
        //TODO:
        left.configurePIDVA(kP, 0, kD, 1 / max_vel, 0);
        right.configurePIDVA(kP, 0, kD, 1 / max_vel, 0);

        // Initialize the timer & Notifier
        stopwatch = new Timer();
        periodicRunnable = new Notifier(new PeriodicRunnable());

        // Start the stopwatch and the notifier (notifier will be called every 0.05 second to ensure trajectory is read properly)
        stopwatch.start();
        periodicRunnable.startPeriodic(0.02);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        periodicRunnable.stop();
        stopwatch.stop();
        Robot.driveTrain.setMotorPercentOutput(0, 0);
        Shuffleboard.putString("Pathfinder", "PathFinder Status", "Command Exited");
        Shuffleboard.putNumber("Pathfinder", "Path Time", stopwatch.get());

        if (first) {
            try {
                FileWriter writer = new FileWriter("/media/sda1/4201Robot/Pathfinder/calibrationFile.txt", true);
                BufferedWriter bufferedWriter = new BufferedWriter(writer);

                bufferedWriter.write("Left Enc. Count: " + Robot.driveTrain.getLeftEncoderCount());
                bufferedWriter.newLine();
                bufferedWriter.write("Right Enc. Count: " + Robot.driveTrain.getRightEncoderCount());

                bufferedWriter.close();
            } catch (Exception e) {
                DriverStation.reportError("Error: Could not write to calibration file", false);
            }
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    class PeriodicRunnable implements Runnable {
        int segment = 0;

        @Override
        public void run() {
            // TODO Auto-generated method stub
            Shuffleboard.putString("Pathfinder", "PathFinder Status", "Running...");

            Shuffleboard.putNumber("Pathfinder", "Segment", segment++);

            // Calculate the current motor outputs based on the trajectory values + encoder positions
            double l = left.calculate(Robot.driveTrain.getLeftEncoderCount());
            double r = right.calculate(Robot.driveTrain.getRightEncoderCount());
            Shuffleboard.putNumber("Pathfinder", "PathFinder L", l);
            Shuffleboard.putNumber("Pathfinder", "PathFinder R", r);
            Shuffleboard.putNumber("Pathfinder", "PathFinder H", Pathfinder.r2d(left.getHeading()));

            // Adjust a turn value based on the gyro's heading + the trajectory's heading. Note that we only use the left's heading, but left/right would be the same since they're following the same path, but separated by wheelbase distance.
            double angleDifference = 0;
            double turn = 0;

            try {
                angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()) + Robot.driveTrain.navX.getAngle());
                turn = 2 * (-1.0 / 80.0) * angleDifference;
            } catch (Exception e) {
                angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));
                turn = 2 * (-1.0 / 80.0) * angleDifference;
            }

            Shuffleboard.putNumber("Pathfinder", "PathFinder T", turn);
            Shuffleboard.putNumber("Pathfinder", "PathFinder L output", l + turn);
            Shuffleboard.putNumber("Pathfinder", "PathFinder R output", r - turn);

            Shuffleboard.putNumber("Pathfinder", "Timer", stopwatch.get());

            // Set the output to the motors
            Robot.driveTrain.setMotorPercentOutput(l + turn, r - turn);

            // Continue sending output values until the path has been completely followed.
            if (left.isFinished() && right.isFinished() && (Math.abs(angleDifference) < 4)) {
                end = true;
            }
        }
    }
}
