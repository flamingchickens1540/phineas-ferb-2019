package org.team1540.robot2019.utils;

public class ControlUtils {

    public static double velocityPosNegConstrain(double velocity, double maxVelocity, double minVelocity) {
        if (Math.abs(velocity) > maxVelocity) {
            velocity = Math.copySign(maxVelocity, velocity);
        } else if (Math.abs(velocity) < minVelocity) {
            velocity = Math.copySign(minVelocity, velocity);
        }
        return velocity;
    }
}
