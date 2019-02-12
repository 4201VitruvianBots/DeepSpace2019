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
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;

    public static int leftJoystick = 0;
    public static int rightJoystick = 1;
    public static int xBoxController = 2;

    public static int pdp = 0;
    public static int PCMOne = 11;
        public static int driveTrainShifterForward = 0;
        public static int driveTrainShifterReverse = 1;
        public static int hatchIntakeExtendForward = 2;
        public static int hatchIntakeExtendReverse = 3;
        public static int hatchIntakeSecureForward = 4;
        public static int hatchIntakeSecureReverse = 5;

    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21;
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;

    public static int leftElevator = 30;
    public static int rightElevator = 31;

    public static int wristMotor = 40;

    public static int cargoIntakeMotor = 60;
    public static int hatchIntakeMotor = 61;

    // DIOs
    public static int bannerIR = 0;
    public static int hatchSensor = 1;
    public static int elevatorBottom = 2;
    public static int elevatorTop = 3;
    public static int wristBottom = 4;
    public static int wristTop = 5;

    // PDP Channels
    public static int pdpChannelDriveTrainLeftForward = 14;
    public static int pdpChannelDriveTrainLeftReverse = 15;
    public static int pdpChannelDriveTrainRightForward = 1;
    public static int pdpChannelDriveTrainRightReverse = 0;
    public static int pdpChannelElevatorLeft = 13;
    public static int pdpChannelElevatorRight = 2;

    // Important Constants
    public static double WRIST_RETRACTED_ANGLE = 0;
    public static double WRIST_EXTENDED_ANGLE = 0;
    public static double WRIST_CARGO_HIGH_ANGLE = 0;
    public static double WRIST_HATCH_ANGLE = 0;
    public static double ELEVATOR_HOME_POSITION = 0;
    public static double ELEVATOR_HATCH_LOW_POSITION = 0;
    public static double ELEVATOR_HATCH_MID_POSITION = 0;
    public static double ELEVATOR_HATCH_HIGH_POSITION = 0;
    public static double ELEVATOR_HATCH_GROUND_LOW_POSITION = 0;
    public static double ELEVATOR_HATCH_GROUND_MID_POSITION = 0;
    public static double ELEVATOR_HATCH_GROUND_HIGH_POSITION = 0;
    public static double ELEVATOR_CARGO_SHIP_POSITION = 0;
    public static double ELEVATOR_CARGO_LOW_POSITION = 0;
    public static double ELEVATOR_CARGO_MID_POSITION = 0;
    public static double ELEVATOR_CARGO_HIGH_POSITION = 0;

    // Pathfinder (Units in feet)
    public static double wheel_diameter = 0.5104167;
    public static double wheelbase = 2.20833;
    public static double max_vel_high = 0.5104167;
    public static double max_vel_low = 1.6;
}
