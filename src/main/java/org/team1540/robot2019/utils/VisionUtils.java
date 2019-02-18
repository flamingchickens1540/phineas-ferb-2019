package org.team1540.robot2019.utils;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class VisionUtils {

    public static boolean isWithinBounds(Vector2D point, double yMax, double yMin, double xMax, double xMin) {
        return !(point.getY() > yMax || point.getY() < yMin || point.getX() > xMax || point.getX() < xMin);
    }
}
