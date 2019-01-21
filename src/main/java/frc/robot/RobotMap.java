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

    public static int PCMOne = 11;
        public static int driveTrainShifterForward = 0;
        public static int driveTrainShifterReverse = 1;
        // TODO: Update when solenoid is fixed
        public static int hatchIntakeForward = 4;
        public static int hatchIntakeReverse = 5;
     //  public static int driveTrainShifterReverse = 1;
     //  public static int driveTrainShifterReverse = 1;

    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21;
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;


    // Pathfinder
    public static double wheel_diameter = 0.5104167;
    public static double wheelbase = 2.20833;
    public static double max_vel_high = 0.5104167;
    public static double max_vel_low = 1.6;
}
