package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.commands.leds.StatusLEDs;
import org.team1540.rooster.wrappers.RevBlinken;

public class LEDs extends Subsystem {
    public void set(RevBlinken.ColorPattern colorPattern) {
        Hardware.leds.set(colorPattern);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new StatusLEDs());
    }
}
