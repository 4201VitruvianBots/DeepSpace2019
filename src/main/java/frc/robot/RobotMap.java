/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // global variables
    public static int mechanismPositions;
    // Constants

    public static int leftJoystick = 0;
    public static int rightJoystick = 1;
    public static int xBoxController = 2;

    // Pneumatics
    public static int pdp = 0;
    public static int PCMOne = 11;
        public static int driveTrainShifterForward = 0;
        public static int driveTrainShifterReverse = 1;
        public static int hatchIntakeExtendForward = 2;
        public static int hatchIntakeExtendReverse = 3;
        public static int hatchIntakeSecureForward = 4;
        public static int hatchIntakeSecureReverse = 5;
        public static int climbPistonForward = 6;
        public static int climbPistonReverse = 7;

    // Drivetrain
    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21;
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;

    public static int leftElevator = 30;
    public static int rightElevator = 31;

    public static int wristMotor = 40;
    public static int climbMotor = 50;

    public static int cargoIntakeMotor = 60;
    public static int hatchIntakeMotor = 61;

    // DIOs
    public static int bannerIR = 0;
    public static int hatchSensor = 1;
    public static int elevatorBottom = 2;
    public static int elevatorTop = 3;
    public static int elevatorMid = 4;
    public static int wristBottom = 5;
    public static int wristTop = 6;
    public static int robotSwitch = 9;

    public static int ledCh0 = 23;  //LED info channels start from 6 on NavX    (blue)
    public static int ledCh1 = 22;  //and go up                                 (magenta)
    public static int ledCh2 = 21;  //                                          (red)
    public static int ledCh3 = 20;  //                                          (green)

    // PDP Channels
    public static int pdpChannelDriveTrainLeftForward = 14;
    public static int pdpChannelDriveTrainLeftReverse = 15;
    public static int pdpChannelDriveTrainRightForward = 1;
    public static int pdpChannelDriveTrainRightReverse = 0;
    public static int pdpChannelElevatorLeft = 13;
    public static int pdpChannelElevatorRight = 2;

    // Setpoints (Units in inches or degrees)
    public static double WRIST_RETRACTED_ANGLE = 135;
    public static double WRIST_EXTENDED_ANGLE = 0;
    public static double WRIST_CARGO_ANGLE = 20;
    public static double WRIST_CARGO_INTAKE_STATION_ANGLE = 90;
    public static double WRIST_CARGO_HIGH_ANGLE = 40;
    public static double WRIST_HATCH_LOW_ANGLE = 45;
    public static double WRIST_HATCH_ANGLE = 85;
    public static double ELEVATOR_HOME_POSITION = 0;
    public static double ELEVATOR_HATCH_INTAKE_POSITION = 10.5;
    public static double ELEVATOR_HATCH_LOW_POSITION = 10;
    public static double ELEVATOR_HATCH_MID_POSITION = 38;
    public static double ELEVATOR_HATCH_HIGH_POSITION = 64;
    public static double ELEVATOR_HATCH_GROUND_LOW_POSITION = 0;
    public static double ELEVATOR_HATCH_GROUND_MID_POSITION = 20;
    public static double ELEVATOR_HATCH_GROUND_HIGH_POSITION = 47;
    public static double ELEVATOR_CARGO_INATAKE_GROUND_POSITION = 2.75;
    public static double ELEVATOR_CARGO_INATAKE_STATION_POSITION = 10.5;
    public static double ELEVATOR_CARGO_SHIP_POSITION = 25;
    public static double ELEVATOR_CARGO_LOW_POSITION = 16;
    public static double ELEVATOR_CARGO_MID_POSITION = 44;
    public static double ELEVATOR_CARGO_HIGH_POSITION = 63;

    // Pathfinder (Units in feet)
    public static double wheel_diameter = 0.5104167;
    public static double wheelbase = 2.20833;
    public static double max_vel_high = 0.5104167;
    public static double max_vel_low = 1.6;
}
