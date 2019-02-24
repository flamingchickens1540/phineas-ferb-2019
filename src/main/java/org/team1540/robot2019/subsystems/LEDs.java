package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;

public class LEDs extends Subsystem {

    public void setColor(LEDColor color) {
        Hardware.redLEDs.set(color.red);
        Hardware.blueLEDs.set(color.blue);
    }

    @Override
    protected void initDefaultCommand() {

    }

    public enum LEDColor {
        RED(false, true), BLUE(true, false), PURPLE(true, true), OFF(false, false);

        final boolean blue;
        final boolean red;

        LEDColor(boolean blue, boolean redOn) {
            this.blue = blue;
            this.red = redOn;
        }
    }
}
