package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import static org.team1540.robot2019.Hardware.redLEDs;
import static org.team1540.robot2019.Hardware.blueLEDs;

public class LEDs extends Subsystem {

    public void red() {
        blueLEDs.set(false);
        redLEDs.set(true);
    }

    public void blue() {
        blueLEDs.set(true);
        redLEDs.set(false);
    }

    public void purple() {
        blueLEDs.set(true);
        redLEDs.set(true);
    }

    public void off() {
        blueLEDs.set(false);
        redLEDs.set(false);
    }

    @Override
    protected void initDefaultCommand() {

    }

}
