package org.team1540.robot2019.subsystems;

import static org.team1540.robot2019.Hardware.hatchGrabber;
import static org.team1540.robot2019.Hardware.hatchSlide;

import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;

public class HatchMech extends Subsystem {

    private static final Logger logger = Logger.getLogger(Wrist.class);

    private volatile boolean extendFlag = false;

    public HatchMech() {
        Hardware.hatchExtendSwitch.requestInterrupts(new InterruptHandlerFunction<>() {
            @Override
            public void interruptFired(int i, Object o) {
                logger.debug("Hatch Extend Interrupt");
                extendFlag = true;
            }
        });

        Hardware.hatchExtendSwitch.setUpSourceEdge(false, true);
        Hardware.hatchExtendSwitch.enableInterrupts();
    }

    public boolean wasExtendSensorTripped() {
        return extendFlag;
    }

    public boolean clearExtendSensor() {
        if (extendFlag) {
            extendFlag = false;
            return true;
        } else {
            return false;
        }
    }

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

    public boolean isReleased() {
        return hatchGrabber.get();
    }

    public boolean isRetracted() {
        return !hatchSlide.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

}
