/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.LED_CHANNELS;
import frc.robot.RobotMap.LED_COLORS;
import frc.robot.commands.UpdateLEDState;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LEDOutput extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /* States
       4 (): Vision Target aligned
       3 (): Hatch detected
       2 (): BannerIR tripped
       1 (Green):
       0 (default): LEDs default state
     */
    public static int LEDColour = 0;
//    public boolean getShifterState = false;
    public static boolean climbState = false;

    private DigitalOutput[] digitalOutput = {  //array that creates digitalOutput0-4, I think.
        new DigitalOutput(LED_CHANNELS.CH0),  //actual pin numbers defined in RobotMap
        new DigitalOutput(LED_CHANNELS.CH1),
        new DigitalOutput(LED_CHANNELS.CH2),
        new DigitalOutput(LED_CHANNELS.CH3),
    };

    public LEDOutput(){
        super("LED Output");
    }

    public void setPinOutput(boolean state, int pin){
        digitalOutput[pin].set(state);  //uses the digitalOutput to actually write the new state
    }

    public void updateLEDState() {
        if(climbState)
            LEDColour = LED_COLORS.RED;
        else if(Robot.m_oi.rightButtons[1].get())
            LEDColour = LED_COLORS.GREEN;
        else if(Robot.vision.isValidTarget())
            LEDColour = LED_COLORS.BLUE;
        else
            LEDColour = LED_COLORS.YELLOW;
        
        setPinOutput(Intake.intakeState == 2 ? true : false, 0);	// Set first bit for chasing pattern if in cargo intake mode
		setPinOutput((LEDColour % 8 > 3), 1);			  			// checks what each digit of the state # is
		setPinOutput((LEDColour % 4 > 1), 2);  						// in binary, with pin 1 as a 4s place, 2 as
		setPinOutput((LEDColour % 2 > 0), 3);  						// 2s, and 3 as 1s. Pin on for 1 & off for 0.
																	// If # is > 7, the binary # overflows.
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new UpdateLEDState());
    }
}

