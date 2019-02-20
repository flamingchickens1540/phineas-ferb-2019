package org.team1540.robot2019.odometry;


import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;

public class TankDriveOdometryUtils {

    public static Transform3D calcDeltaTransformFromTankDriveDistances(double deltaDistanceLeft, double deltaDistanceRight, double deltaAngle) {
        double deltaX;
        double deltaY;

        if (deltaAngle == 0) { // If the robot has not turned, it has only moved in the X direction
            deltaX = (deltaDistanceLeft + deltaDistanceRight) / 2;
            deltaY = 0;
        } else { // Otherwise, calculate the radius of the turn and calculate the delta position
            double radiusFromLeftArc = TrigUtils.radiusFromArcAndAngle(deltaDistanceLeft, deltaAngle);
            double radiusFromRightArc = TrigUtils.radiusFromArcAndAngle(deltaDistanceRight, deltaAngle);

            double avgRadius = (radiusFromLeftArc + radiusFromRightArc) / 2;

            deltaX = calcDeltaY(avgRadius, deltaAngle);
            deltaY = calcDeltaX(avgRadius, deltaAngle);
        }

        return new Transform3D(deltaX, deltaY, deltaAngle);
    }

    public static double calcDeltaX(double radius, double deltaRads) {
        return radius * (1.0 - Math.cos(deltaRads));
    }

    public static double calcDeltaY(double radius, double deltaRads) {
        return radius * Math.sin(deltaRads);
    }
}
