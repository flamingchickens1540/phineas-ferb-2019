package org.team1540.robot2019.utils;


import org.team1540.robot2019.datastructures.threed.Transform3D;

public class TankDriveOdometryAccumulator {

  private Transform3D odomToBaseLink = Transform3D.IDENTITY;

  private double distancePrevLeft = 0;
  private double distancePrevRight = 0;
  private double angleRadsPrev = 0;

  /**
   * @param distanceLeft Absolute distance in meters of left wheels
   * @param distanceRight Absolute distance in meters of right wheels
   * @param continuousAngle Continuous absolute angle in radians (should NOT jump from 2PI to 0)
   */
  public void update(double distanceLeft, double distanceRight, double continuousAngle) { // TODO: This angle should not need to be continuous
    double deltaDistanceLeft = distanceLeft - distancePrevLeft;
    double deltaDistanceRight = distanceRight - distancePrevRight;
    double deltaRads = continuousAngle - angleRadsPrev;

    distancePrevLeft = distanceLeft;
    distancePrevRight = distanceRight;
    angleRadsPrev = continuousAngle;

    Transform3D deltaDistance = TankDriveOdometry.calcDeltaTransformFromTankDriveDistances(deltaDistanceLeft, deltaDistanceRight, deltaRads);

    odomToBaseLink = odomToBaseLink.add(deltaDistance); // TODO: Should the absolute angle replace the calculated one?
  }

  public Transform3D getTransform() {
    return odomToBaseLink;
  }

  public void reset() {
    distancePrevLeft = 0;
    distancePrevRight = 0;
    angleRadsPrev = 0;
  }
}
