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

    public static double simpleDeadzone(double input, double deadzone) {
        if (Math.abs(input) < Math.abs(deadzone)) {
            return 0;
        }
        return input;
    }

    public static double allVelocityConstraints(double output, double max, double min, double deadzone) {
        return simpleDeadzone(velocityPosNegConstrain(output, max, min), deadzone);
    }
}
