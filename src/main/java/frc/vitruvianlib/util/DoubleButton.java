package frc.vitruvianlib.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.buttons.Button;

public class DoubleButton extends Button {
    private GenericHID m_joystick1, m_joystick2;
    private int m_buttonNumber1, m_buttonNumber2;


    public DoubleButton(GenericHID joystick, int buttonNumber1, int buttonNumber2) {
        m_joystick1 = joystick;
        m_joystick2 = joystick;
        m_buttonNumber1 = buttonNumber1;
        m_buttonNumber2 = buttonNumber2;
    }

    public DoubleButton(GenericHID joystick1, int buttonNumber1, GenericHID joystick2, int buttonNumber2) {
        m_joystick1 = joystick1;
        m_joystick2 = joystick2;
        m_buttonNumber1 = buttonNumber1;
        m_buttonNumber2 = buttonNumber2;
    }


    @Override
    public boolean get() {
        return m_joystick1.getRawButton(m_buttonNumber1) && m_joystick2.getRawButton(m_buttonNumber2);
    }
}
