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
  public static int state = 0;

  private DigitalOutput[] digitalOutput = {  //array that creates digitalOutput0-4, I think.
        new DigitalOutput(RobotMap.ledCh0),  //actual pin numbers defined in RobotMap
        new DigitalOutput(RobotMap.ledCh1),
        new DigitalOutput(RobotMap.ledCh2),
        new DigitalOutput(RobotMap.ledCh3),
    };
  private boolean[] DIOState = {  //array that creates DIOState0-4
        false,       //initialises variables for all of the pins to false
        false,
        false,
        false,
  };

  public LEDOutput(){
      super("LED Output");
  }

  public void setPinOutput(boolean state, int pin){
    digitalOutput[pin].set(state);  //uses the digitalOutput to actually write the new state
    DIOState[pin] = state;          //sets our variable to the state so we know what the pin is when we want it below
  }

  public boolean getDIOState(int pin){ return DIOState[pin]; }  //returns the value of the pin, used for toggles & the like

  public void updateLEDState() {  //called in RobotPeriodic to, well, update LED state.
    if(Robot.vision.getTargetArea() > 0.85 && Math.abs(Robot.vision.getTargetX()) < 0.1) {  //if the target is in the centre and is big, light up blue
      state = 3;
    } else if(Robot.vision.isValidTarget()) { //if the limelight detects any target at all, light up magenta
      state = 2;
    } else if(Robot.intake.bannerIR.get()) {  //if the cargo is detected by the banner IR, light up green
      state = 0;
    } else if(Robot.wrist.getAngle()<3 && Robot.elevator.getPosition()<3260) {  //if the elevator's < 5" and wrist is < 3 deg, light up red
      state = 1;
    } else {
      state = 0;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LEDReaction());
  }
}

