package frc.vitruvianlib.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

public class MultiButton extends Button {
    private Button[] buttonArray;
    private int size;

    public MultiButton(Button... buttons) {
        size = buttons.length;
        buttonArray = new Button[size];
        int i = 0;
        for(Button button:buttons)
            buttonArray[i++] = button;
    }

    @Override
    public boolean get() {
        for(Button button: buttonArray) {
            if(button.get() == false)
                return false;
        }
        return true;
    }
}
