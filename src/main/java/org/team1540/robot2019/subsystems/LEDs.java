package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.commands.leds.StatusLEDs;

public class LEDs extends Subsystem {

    public void setColor(LEDColor color) {
        Hardware.redLEDs.set(color.red);
        Hardware.blueLEDs.set(color.blue);
    }

    public void setRaw(boolean red, boolean blue, boolean green) {
        Hardware.redLEDs.set(red);
        Hardware.blueLEDs.set(blue);
        Hardware.greenLEDs.set(green);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new StatusLEDs());
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
