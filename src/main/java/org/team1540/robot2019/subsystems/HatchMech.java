package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.hatchGrabber;
import static org.team1540.robot2019.Hardware.hatchSlide;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchMech extends Subsystem {

    public void extend() {
        hatchSlide.set(true);
    }

    public void retract() {
        hatchSlide.set(false);
    }

    public void grab() {
        hatchGrabber.set(false);
    }

    public void release() {
        hatchGrabber.set(true);
    }

    @Override
    protected void initDefaultCommand() {
    }

}
