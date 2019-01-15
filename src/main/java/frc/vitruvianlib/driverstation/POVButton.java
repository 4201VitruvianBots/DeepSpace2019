package frc.vitruvianlib.driverstation;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * An interface to turn the POV arrow keys into a button, as they are treated as
 * a variable value from 0-360 based on which button(s) you press, rather than a
 * boolean like a normal button.
 */
public class POVButton extends Button {

    private Joystick joy;
    private int pov;

    /**
     * Create a trigger for running a command from a POV direction and up to two alternates
     *
     * @param joy Joystick to use
     * @param pov POV to use
     */
    public POVButton(Joystick joy, int pov) {
        this.joy = joy;
        this.pov = pov;
    }

    public boolean get() {
        int current = joy.getPOV(pov);
        return current == pov && joy.getPOVCount() > 0;
    }
}