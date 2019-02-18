package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import static org.team1540.robot2019.Hardware.greenLEDs;
import static org.team1540.robot2019.Hardware.redLEDs;

public class LEDs extends Subsystem {

    public void red() {
        redLEDs.set(true);
        greenLEDs.set(false);
    }

    public void green() {
        redLEDs.set(false);
        greenLEDs.set(false);
    }

//    public void yellow() {
//        redLEDs.set(true);
//        greenLEDs.set(true);
//    }

    public void off() {
        redLEDs.set(false);
        greenLEDs.set(false);
    }

    @Override
    protected void initDefaultCommand() {

    }

}
