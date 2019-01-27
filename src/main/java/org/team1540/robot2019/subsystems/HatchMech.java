package org.team1540.robot2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import static org.team1540.robot2019.Hardware.hatchSlide;
import static org.team1540.robot2019.Hardware.hatchSuctionCups;

public class HatchMech extends Subsystem {

    public void getHatch() {
        slideOut();
        attatch();
    }

    public void placeHatch() {
        release();
        slideIn();
    }

    public void slideOut() {
        hatchSlide.set(true);
    }

    public void slideIn() {
        hatchSlide.set(false);
    }

    public void attatch() {
        hatchSuctionCups.set(true);
    }

    public void release() {
        hatchSuctionCups.set(false);
    }

    @Override
    protected void initDefaultCommand() {
    }

}
