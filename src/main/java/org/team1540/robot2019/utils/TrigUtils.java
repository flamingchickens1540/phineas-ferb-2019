package org.team1540.robot2019.utils;

public class TrigUtils {

  public static double SignedAngleDifference(double angle1, double angle2) {
    double diff = (angle2 - angle1 + Math.PI) % (Math.PI * 2) - Math.PI;
    return diff < -Math.PI ? diff + (Math.PI * 2) : diff;
  }

  public static double radiusFromArcAndAngle(double arcLength, double centralAngle) {
    return arcLength / centralAngle;
  }
}
