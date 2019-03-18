package org.team1540.robot2019.odometry.tankdrive;


import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;

public class TankDriveOdometryAccumulator {

    private Transform3D odomToBaseLink = Transform3D.IDENTITY;

    private double distancePrevLeft = 0;
    private double distancePrevRight = 0;
    private double angleRadsPrev = 0;

    private boolean firstUpdate = true;

    /**
     * @param distanceLeft Absolute distance in meters of left wheels
     * @param distanceRight Absolute distance in meters of right wheels
     * @param angleRadians Right-handed rotation about Z axis (up)
     */
    public void update(double distanceLeft, double distanceRight, double angleRadians) {
        if (firstUpdate) { // TODO: Prevent having to reset distances
            distancePrevLeft = distanceLeft;
            distancePrevRight = distanceRight;
            firstUpdate = false;
        }

        double deltaDistanceLeft = distanceLeft - distancePrevLeft;
        double deltaDistanceRight = distanceRight - distancePrevRight;
        double deltaRads = TrigUtils.signedAngleError(angleRadsPrev, angleRadians);

        distancePrevLeft = distanceLeft;
        distancePrevRight = distanceRight;
        angleRadsPrev = angleRadians;

        Transform3D deltaDistance = TankDriveOdometryUtils.calcDeltaTransformFromTankDriveDistances(deltaDistanceLeft, deltaDistanceRight, deltaRads);

        odomToBaseLink = odomToBaseLink.add(deltaDistance); // TODO: Should the absolute angle replace the calculated one?
    }

    public Transform3D getTransform() {
        return odomToBaseLink;
    }
}
