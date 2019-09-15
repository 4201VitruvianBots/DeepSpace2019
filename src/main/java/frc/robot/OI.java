/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.harpoon.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.test.*;
import frc.vitruvianlib.driverstation.XBoxTrigger;
import frc.vitruvianlib.util.MultiButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
    Joystick leftJoystick = new Joystick(RobotMap.leftJoystick);
    Joystick rightJoystick = new Joystick(RobotMap.rightJoystick);
    Joystick xBoxController = new Joystick(RobotMap.xBoxController);
    public Button[] leftButtons = new Button[7];
    public Button[] rightButtons = new Button[7];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;
    public MultiButton enableClimbButton, disableClimbButton;

    /* Indicators for which setpoint you selected
     * 0: Stowed
     * 1: Intake
     * 2: Cargo Ship
     * 3: Rocket Low
     * 4: Rocket Mid
     * 5: Rocket High
     */
    public static int positionIndex = 0;
    boolean[] positionIndicator = {false, false, false, false, false, false};

    public OI() {
        initializeButtons();
    }

    void initializeButtons() {
        for (int i = 0; i < leftButtons.length; i++)
            leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightButtons.length; i++)
            rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
        for (int i = 0; i < xBoxButtons.length; i++)
            xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
        for (int i = 0; i < xBoxPOVButtons.length; i++)
            xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
        //xBoxPOVButtons[0] = new POVButton(xBoxController, 0);
        //xBoxPOVButtons[1] = new POVButton(xBoxController, 90);

        xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
        xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);
        enableClimbButton = new MultiButton(leftButtons[2],xBoxRightTrigger);
        disableClimbButton = new MultiButton(rightButtons[2],xBoxButtons[5]);

        /*  Left Joystick Buttons:
            3 - Right Button: Set DriveTrain Low Gear
            4 - Left Button: Set DriveTrain High Gear
        */
        //leftButtons[0].whileHeld(new IntakeIntake());
        leftButtons[3].whenPressed(new SetDriveShifters(true));
        leftButtons[4].whenPressed(new SetDriveShifters(false));

        /*  Right Joystick Buttons:
            0 - Trigger: Release/Score Game Piece
            1 - Center Button: Use Limelight drive assist
        */

        rightButtons[0].whileHeld(new IntakeRelease());
        rightButtons[1].whileHeld(new FollowVisionTarget());
//        rightButtons[1].whileHeld(new AutoSteer());
        // TODO: Test this version of limelight auto-correction
        //rightButtons[1].whileHeld(new HoldToAlignWithTarget());

        //rightButtons[1].whenPressed(new TurnToAngle(180));
        //rightButtons[2].whenPressed(new TurnToAngle(-90));
        //rightButtons[3].whenPressed(new TurnToAngle(90));

        /*  xBox Controller Buttons:
            0 - A Button: Set Mechanisms to Rocket Low
            1 - B Button: Set Mechanisms to Rocket Medium
            3 - Y Button: Set Mechanisms to Rocket High
            2 - X Button: Set Mechanisms to Cargo Ship
        */

        /* Game Piece
        2 Cargo
        1 Hatch Ground
        0 Hatch
        */

        /* Position
        5 Rocket High
        4 Rocket Mid
        3 Rocket Low
        2 Cargo ship
        1 intake
        0 home
        -1 Depot
        */

        xBoxButtons[0].whenPressed(new SetAllMechanismSetpoints(3));
        xBoxButtons[1].whenPressed(new SetAllMechanismSetpoints(4));
        xBoxButtons[2].whenPressed(new SetAllMechanismSetpoints(2));
        xBoxButtons[3].whenPressed(new SetAllMechanismSetpoints(5));
        
        /*  xBox Controller Buttons:
         	Cargo Mode:
	        LeftTrigger: Move to Cargo Ground Setpoint & Intake
	        4 Left Button: Move to Cargo Depot Setpoint & Intake
	        
         	Cargo Mode:
	        LeftTrigger: Move to Harpoon Intake Setpoint & Intake
	        4 Left Button: Extend Harpoon
         */

        xBoxLeftTrigger.whileHeld(new ConditionalCommand(new IntakeIntake(), new HoldHarpoonExtend()) {
            @Override
            protected boolean condition() {
                return Robot.intake.intakeState == 2;
            }
        });
        xBoxLeftTrigger.whenPressed(new SetAllMechanismSetpoints(1));
        xBoxButtons[4].whileHeld(new IntakeIntake());
        xBoxButtons[4].whenPressed(new SetAllMechanismSetpoints(-1));
        
        /*  xBox Controller Buttons:
	        RightTrigger: Select Intake Cargo State
	        5 - Right Button: Select Intake Hatch State
	
	        POV UP: Zero all encoders
	        POV Left, Right, Down: Set stowed position
	        6 - Start (?): Toggle Elevator Control Mode
	        7 - Select (?): Toggle Wrist Control Mode
	     */
        
        xBoxRightTrigger.whenPressed(new SetIntakeState(2));
        xBoxButtons[5].whenPressed(new SetIntakeState(0));

        xBoxButtons[6].whenPressed(new ToggleElevatorState()); //elevator
        xBoxButtons[7].whenPressed(new ToggleWristState()); //wrist
        xBoxPOVButtons[0].whenPressed(new ZeroElevatorEncoder());
        xBoxPOVButtons[0].whenPressed(new ZeroWristEncoder());
        for(int i = 1; i < xBoxPOVButtons.length; i++)
            xBoxPOVButtons[i].whenPressed(new SetAllMechanismSetpoints(0));
//        xBoxPOVButtons[2].whenPressed(new SetAllMechanismSetpoints(0));

        //leftButtons[1].whenPressed(new ResetNavXAngle());
        //leftButtons[2].whenPressed(new TestControllerRumble(leftJoystick, 3));
        //rightButtons[2].whenPressed(new TestControllerRumble(rightJoystick, 3));

        //xBoxButtons[0].whileHeld(new IntakeControl(true));
        //xBoxButtons[1].whileHeld(new IntakeControl(false));
        
        /* Climb Mode Enable: Both the center button on the left joystick and the right trigger 
         * on the xBox controller must be pressed in order to enable climb mode
         * Climb Mode Disable: Both the center button on the right joystick and the right button 
         * on the xBox controller must be pressed in order to disable climb mode
         */
        enableClimbButton.whenPressed(new EnableClimbSequence());
        disableClimbButton.whenPressed(new DisableClimbSequence());
    }

    public void updateSetpointIndicator(){
        for(int i = 0; i < positionIndicator.length; i++)
            positionIndicator[i] = false;
        positionIndicator[Math.abs(positionIndex)] = true;
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Rocket High", positionIndicator[5]);
        SmartDashboard.putBoolean("Rocket Mid", positionIndicator[4]);
        SmartDashboard.putBoolean("Rocket Low", positionIndicator[3]);
        SmartDashboard.putBoolean("Cargo Ship", positionIndicator[2]);
        SmartDashboard.putBoolean("Intake", positionIndicator[1]);
        SmartDashboard.putBoolean("Stowed", positionIndicator[0]);
    }

    public double getLeftJoystickX() {
        return leftJoystick.getX();
    }

    public double getLeftJoystickY() {
        return -leftJoystick.getY();
    }

    public double getLeftJoystickZ() {
        return -leftJoystick.getZ();
    }

    public double getLeftRotation(){
        return leftJoystick.getZ();
    }
    public double getRightJoystickX() {
        return rightJoystick.getX();
    }

    public double getRightJoystickY() {
        return -rightJoystick.getY();
    }

    public double getRightJoystickZ() {
        return -rightJoystick.getY();
    }

    public double getRawLeftJoystickX() {
        return leftJoystick.getX();
    }

    public double getRawLeftJoystickY() {
        return -leftJoystick.getY();
    }

    public double getRawRightJoystickX() {
        return rightJoystick.getX();
    }

    public double getRawRightJoystickY() {
        return -rightJoystick.getY();
    }

    public double getXBoxLeftX(){
        return xBoxController.getRawAxis(0);
    }

    public double getXBoxLeftY(){
        return -xBoxController.getRawAxis(1);
    }

    public double getXBoxRightX(){
        return xBoxController.getRawAxis(4);
    }

    public double getXBoxRightY(){
        return -xBoxController.getRawAxis(5);
    }

    public void enableXBoxRumbleTimed(double duration){
        Thread t = new Thread(() -> {
            setXBoxRumble(0.8);
            Timer.delay(duration);
            setXBoxRumble(0);
        });
        t.start();
    }

    private void setXBoxRumble(double value) {
        xBoxController.setRumble(GenericHID.RumbleType.kLeftRumble, value);
        xBoxController.setRumble(GenericHID.RumbleType.kRightRumble, value);
    }
}
