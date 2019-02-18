package org.team1540.robot2019.utils;

import org.team1540.robot2019.Hardware;

public class NavxWrapper {

    /**
     * @return Navx yaw counter-clockwise in radians
     */
    public double getYawRadians() {
        return -Math.toRadians(Hardware.navx.getYaw());
    }

    public double getAngleRadians() {
        return -Math.toRadians(Hardware.navx.getAngle());
    }
}
