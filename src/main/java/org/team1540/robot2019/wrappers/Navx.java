package org.team1540.robot2019.wrappers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;

public class Navx {

    private final AHRS navx;

    public Navx(Port port) {
        navx = new AHRS(port);
    }

    /**
     * @return Navx yaw counter-clockwise in radians
     */
    public double getYawRadians() {
        return -Math.toRadians(navx.getYaw());
    }

    /**
     * @return Navx angle counter-clockwise in radians
     */
    public double getAngleRadians() {
        return -Math.toRadians(navx.getAngle());
    }

    /**
     * This is bad. Do NOT use this. Re-tune your PID if you have to.
     *
     * @return Navx raw pitch clockwise in degrees
     */
    public double getRawPitchDegrees() {
        return navx.getPitch();
    }
}
