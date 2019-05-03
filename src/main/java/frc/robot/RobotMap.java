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

    public static class PDP {
    	public static int CAN_ADDRESS = 0;
    	public static int DRIVETRAIN_LEFT_FRONT = 14;
    	public static int DRIVETRAIN_LEFT_REAR = 15;
    	public static int DRIVETRAIN_RIGHT_FRONT = 1;
    	public static int DRIVETRAIN_RIGHT_REAR = 0;
    	public static int ELEVATOR_LEFT_A = 13;
    	public static int ELEVATOR_LEFT_B = 12;
    	public static int ELEVATOR_RIGHT_A = 2;
    	public static int ELEVATOR_RIGHT_B = 3;
    	public static int WRIST = 12;
    	public static int LIMELIGHT = 10;
    }
    
    public static enum PCM_ONE {
    	DRIVETRAIN_SIFTER (0, 1),
    	HATCH_EXTEND (4, 5),
//    	HATCH_SECURE (4, 5),
    	CLIMB_PISTONS (2, 3);
    	
    	public int FORWARD;
    	public int REVERSE;
    	
    	private PCM_ONE(int forward, int reverse){
    		this.FORWARD = forward;
    		this.REVERSE = reverse;
    	}
    	
    	public static int ADDRESS = 11;
    }
    
    public static class LED_CHANNELS {
    	public static int CH0 = 23;	//LED info channels start from 9 on NavX    Arduino port 12     
    	public static int CH1 = 22;	//and go down                               Arduino port 11     
    	public static int CH2 = 21;	//                                          Arduino port 10     
    	public static int CH3 = 20;	//                                          Arduino port 9      
    }
    
    public static class LED_COLORS {
    	public static int BLUE = 1;
    	public static int RED = 2;
    	public static int YELLOW = 3;
    	public static int GREEN = 4;
    }
    
    // Drivetrain
    public static int leftFrontDriveMotor = 20;
    public static int leftRearDriveMotor = 21;
    public static int rightFrontDriveMotor = 22;
    public static int rightRearDriveMotor = 23;
    public static int climbDriveMotor = 24;

    public static int leftElevatorA = 30;
    public static int leftElevatorB = 31;
    public static int rightElevatorA = 32;
    public static int rightElevatorB = 33;

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


    public static class WRIST_SETPOINTS {
	    // Setpoints (Units in inches or degrees)
	    public static double RETRACTED = 165; //165;
	    public static double EXTENDED = 0;
	    public static double CLIMB = -10;
	    public static double CARGO = 45;
	    public static double CARGO_RETRACTED = 135;
	    public static double CARGO_SHIP_INTAKE = 30;
	    public static double CARGO_INTAKE_STATION = 90;
	    public static double CARGO_HIGH = 40;
	    public static double HATCH_LOW = 45;
	    public static double HATCH = 85;
    }
    
    public static class ELEVATOR_SETPOINTS {
	    public static double HOME = 0;
	//    public static double ELEVATOR_HATCH_INTAKE_POSITION = 10.5;
	    //public static double ELEVATOR_HATCH_LOW_POSITION = 10;
	    public static double HATCH_LOW = 7.75;
	    public static double HATCH_MID = 37;
	    public static double HATCH_HIGH = 60;
	    public static double HATCH_GROUND_LOW = 0;
	    public static double HATCH_GROUND_MID = 20;
	    public static double HATCH_GROUND_HIGH = 47;
	    public static double CARGO_INTAKE_DEPOT = 2; // 2
	    public static double CARGO_INTAKE_STATION = 10.5;
	    public static double CARGO_AUTO_INTAKE = 3.5;
	    public static double CARGO_LOW = 10;
	    public static double CARGO_MID = 36;
	    public static double CARGO_HIGH = 63;
	    public static double CARGO_SHIP = 35;
	    public static double CLIMB = 28;
    }
    
    public static class INTAKE_SPEED {
	    public static double CARGO_INTAKE = -0.7; // Change back to -0.7 for comp
	    public static double CARGO_OUTTAKE = 0.7;
	    public static double CARGO_HOLD = -0.2;
	    public static double HATCH_INTAKE = -1;
	    public static double HATCH_OUTTAKE = 1;
	    public static double HATCH_HOLD = 0;
	    public static double HATCH_GROUND_INTAKE = -0.8;
	    public static double HATCH_GROUND_OUTTAKE = 1;
	    public static double HATCH_GROUND_HOLD = -0.1;
    }
    
    // Pathfinder (Units in feet)
    public static double wheel_diameter = 0.5104167;
    public static double wheelbase = 2.20833;
    public static double max_vel_high = 0.5104167;
    public static double max_vel_low = 1.6;
}
