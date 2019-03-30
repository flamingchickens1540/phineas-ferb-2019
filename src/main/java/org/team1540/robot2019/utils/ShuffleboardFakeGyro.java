package org.team1540.robot2019.utils;

import edu.wpi.first.wpilibj.GyroBase;

public class ShuffleboardFakeGyro extends GyroBase {

    private double angle = 0;

    @Override
    public void calibrate() {

    }

    @Override
    public void reset() {

    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public double getRate() {
        return 0;
    }
}
