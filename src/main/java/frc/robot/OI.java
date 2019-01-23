/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.auto.IntakeControl;

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

        //leftButtons[0].whileHeld(new HoldToAlignWithTarget());
        leftButtons[0].whileHeld(new HoldToAlignWithTarget());
        leftButtons[1].whileHeld(new TurnToAngle(-90));
        leftButtons[2].whileHeld(new TurnToAngle(180));
        leftButtons[3].whileHeld(new TurnToAngle(90));

        rightButtons[0].whenPressed(new ToggleHarpoon());
        leftButtons[1].whenPressed(new SetDriveShifters());
        //leftButtons[1].whenPressed(new ResetNavXAngle());
        leftButtons[2].whenPressed(new TestControllerRumble(leftJoystick, 3));
        rightButtons[2].whenPressed(new TestControllerRumble(rightJoystick, 3));

        xBoxButtons[0].whileHeld(new IntakeControl(true));
        xBoxButtons[1].whileHeld(new IntakeControl(false));
    }

    public double getLeftJoystickX() {
        return Math.pow(leftJoystick.getX(), 1);
    }

    public double getLeftJoystickY() {
        return Math.pow(-leftJoystick.getY(), 1);
    }

    public double getRightJoystickX() {
        return Math.pow(rightJoystick.getX(), 1);
    }

    public double getRightJoystickY() {
        return Math.pow(-rightJoystick.getY(), 3);
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

}
