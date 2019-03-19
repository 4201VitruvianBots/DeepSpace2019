/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.InitIntakeHold;
import frc.robot.commands.auto.PathfinderReadLevel1;
import frc.robot.commands.auto.routines.LeftLevel1ToRocket;
import frc.robot.commands.auto.routines.PathfinderCalibration;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.util.*;
import frc.vitruvianlib.VitruvianLogger.VitruvianLogger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static Climber climber = new Climber();
    public static Controls controls = new Controls();
    public static DriveTrain driveTrain = new DriveTrain();
    public static Elevator elevator = new Elevator();
    //public static NerdyElevator nerdyElevator = new NerdyElevator();
    public static IntakeExtend intakeExtend = new IntakeExtend();
    public static Intake intake = new Intake();
    public static Vision vision = new Vision();
    public static Wrist wrist = new Wrist();
    public static LEDOutput ledOutput = new LEDOutput();
    public static OI m_oi;

    Notifier robotPeriodic = new Notifier(new robotPeriodicRunnable());

    boolean shuffleboardTransition = false;

    Command m_autonomousCommand;
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();
    Command m_teleopCommand;
    SendableChooser<Command> m_teleopChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Elevator.initialCalibration = false;
        m_oi = new OI();
        m_autoChooser.addOption("Pathfinder Calibration", new PathfinderCalibration());
        m_autoChooser.addOption("Get Off Level 1", new PathfinderReadLevel1("getOffLevel1"));
        m_autoChooser.addOption("Left Level 1 To Rocket", new LeftLevel1ToRocket());
        SmartDashboard.putData("Auto mode", m_autoChooser);

        m_teleopChooser.setDefaultOption("Arcade Drive", new SetArcadeDrive());
        m_teleopChooser.addOption("Tank Drive", new SetTankDrive());
        m_teleopChooser.addOption("Cheesy Drive", new SetCheesyDrive());
        SmartDashboard.putData("TeleopDrive", m_teleopChooser);

        controls.readIniFile();
        controls.initTestSettings();

        vision.initUSBCamera();

        //if(Elevator.controlMode == 1 && !Elevator.initialCalibration)
        //    Elevator.controlMode = 0;


        //elevator.zeroEncoder();
        //wrist.setAbsolutePosition(RobotMap.WRIST_RETRACTED_ANGLE);

        vision.setPipeline(0);

        // Attempts at stopping the loop time overrun messages
        LiveWindow.disableAllTelemetry();

        robotPeriodic.startPeriodic(0.1);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
//        driveTrain.updateSmartDashboard();
        elevator.updateSmartDashboard();
        wrist.updateSmartDashboard();
        intake.updateSmartDashboard();
//        climber.updateSmartDashboard();
//        m_oi.updateSmartDashboard();
        //vision.updateSmartDashboard();

        // TODO: Enable this when encoders are fixed
        //elevator.zeroEncoder();
        //wrist.zeroEncoder();
//        intake.updateIntakeIndicator();
//        m_oi.updateSetpointIndicator();
//        intake.updateOuttakeState();
//        ledOutput.updateLEDState();
        intake.updateCargoIntakeState();
    }

    public class robotPeriodicRunnable implements Runnable {

        @Override
        public void run() {
            driveTrain.updateShuffleboard();
            elevator.updateShuffleBoard();
            wrist.updateShuffleboard();
            intake.updateShuffleboard();
            vision.updateShuffleboard();
            m_oi.updateSmartDashboard();

            intake.updateIntakeIndicator();
            m_oi.updateSetpointIndicator();
            intake.updateOuttakeState();
            ledOutput.updateLEDState();
        }
    }
    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        shuffleboardTransition = false;
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.stopRecording();
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.startRecording();
        VitruvianLogger.getInstance().startLogger();

        // Default VP Pipeline
        vision.setPipeline(0);
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        Robot.vision.setPipeline(0);
        driveTrain.setDriveMotorsState(true);
        if(Elevator.controlMode == 1)
            elevator.setAbsoluteHeight(elevator.getHeight());
        if(Wrist.controlMode == 1)
            wrist.setAbsolutePosition(wrist.getAngle());

        
        m_autonomousCommand = m_autoChooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        driveTrain.zeroEncoderCounts();
        if (m_autonomousCommand != null) {
//            driveTrain.setDriveMotorsState(false);
            m_autonomousCommand.start();
        }
        Scheduler.getInstance().add(new InitIntakeHold());

        VitruvianLogger.getInstance().startLogger();
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.stopRecording();
        edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.startRecording();
        shuffleboardTransition = true;
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        Robot.vision.setPipeline(0);
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        driveTrain.setDriveMotorsState(true);

        m_teleopCommand = m_teleopChooser.getSelected();
//        if (m_teleopCommand != null)
//            Robot.driveTrain.setDefaultCommand(m_teleopCommand);

        if(Elevator.controlMode == 1)
           elevator.setAbsoluteHeight(elevator.getHeight());
        if(Wrist.controlMode == 1)
            wrist.setAbsolutePosition(wrist.getAngle());

        VitruvianLogger.getInstance().startLogger();

        // Only reset shuffleboard's recording if starting from disabledInit
        if(!shuffleboardTransition) {
            edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.stopRecording();
            edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.startRecording();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
