package frc.vitruvianlib.driverstation;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;


/**
 * An interface to turn the XBox Triggers into a button, as they are treated as
 * a variable throttle rather than a boolean like a normal button.
 */
public class XBoxTrigger extends Button {
    Joystick joystick;
    int axis;

    public XBoxTrigger(Joystick joystick, int axis) {
        this.joystick = joystick;
        this.axis = axis;
    }

    @Override
    public boolean get() {
        return joystick.getRawAxis(axis) > 0.15;
    }

}
