package org.team1540.robot2019.utils;


import org.team1540.robot2019.datastructures.threed.Transform3D;

public class TankDriveOdometryAccumulator {

    private Transform3D odomToBaseLink = Transform3D.IDENTITY;

    private double distancePrevLeft = 0;
    private double distancePrevRight = 0;
    private double angleRadsPrev = 0;

//    private boolean firstUpdate = true;

    /**
     * @param distanceLeft Absolute distance in meters of left wheels
     * @param distanceRight Absolute distance in meters of right wheels
     * @param angleRadians Right-handed rotation about Z axis (up)
     */
    public void update(double distanceLeft, double distanceRight, double angleRadians) {
//        if (firstUpdate) {
//            distancePrevLeft = distanceLeft;
//            distancePrevRight = distanceRight;
//            angleRadsPrev = angleRadians;
//            firstUpdate = false;
//            return;
//        }

        double deltaDistanceLeft = distanceLeft - distancePrevLeft;
        double deltaDistanceRight = distanceRight - distancePrevRight;
        double deltaRads = TrigUtils.signedAngleError(angleRadians, angleRadsPrev);

        distancePrevLeft = distanceLeft;
        distancePrevRight = distanceRight;
        angleRadsPrev = angleRadians;

        Transform3D deltaDistance = TankDriveOdometry.calcDeltaTransformFromTankDriveDistances(deltaDistanceLeft, deltaDistanceRight, deltaRads);

        odomToBaseLink = odomToBaseLink.add(deltaDistance); // TODO: Should the absolute angle replace the calculated one?
    }

    public Transform3D getTransform() {
        return odomToBaseLink;
    }
}
