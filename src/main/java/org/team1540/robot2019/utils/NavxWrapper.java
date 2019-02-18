package org.team1540.robot2019.utils;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

public class NavxWrapper {

    private AHRS navx = new AHRS(Port.kMXP);

    /**
     * @return Navx yaw counter-clockwise in radians
     */
    public double getYawRadians() {
        return -Math.toRadians(navx.getYaw());
    }

    public double getAngleRadians() {
        return -Math.toRadians(navx.getAngle());
    }
}
