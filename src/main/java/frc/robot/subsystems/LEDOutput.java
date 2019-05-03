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
import frc.robot.commands.LEDReaction;

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

    public void updateLEDState() {  //called in RobotPeriodic to, well, update LED state.
//        getShifterState = Robot.driveTrain.getDriveShifterStatus();    //so we can tell if it's in low (false) or lower (true) gear

        if(climbState)
            LEDColour = LED_COLORS.RED;
        else if(Robot.m_oi.rightButtons[1].get())
            LEDColour = LED_COLORS.GREEN;
        else if(Robot.vision.isValidTarget())
            LEDColour = LED_COLORS.BLUE;
        else
            LEDColour = LED_COLORS.YELLOW;

//        if(Robot.wrist.controlMode == 0){ //if wrist is in manual mode
//            LEDColour = 1;
//        }
//        else if(Intake.intakeState == 2){   //if robot is in cargo intake mode
//            LEDColour = 2;
//        }
//      /*  else if(truen't){ //This spot reserved for current-spike hatch intake detection
//            LEDColour = 4;
//        }*/
//        else if (Intake.intakeState <2){    //if robot is in hatch intake mode
//            LEDColour = 3;
//        }
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new LEDReaction());
    }
}

