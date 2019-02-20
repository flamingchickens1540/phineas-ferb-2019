package org.team1540.robot2019.datastructures.utils;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class AboutEquals {

    public static boolean rotationAboutEquals(Rotation a, Rotation b, double tolerance) {
        Rotation c = a.applyInverseTo(b);
        return Math.abs(c.getQ0()) - 1 <= tolerance &&
            Math.abs(c.getQ1()) <= tolerance &&
            Math.abs(c.getQ2()) <= tolerance &&
            Math.abs(c.getQ3()) <= tolerance;
    }

    public static boolean vector3DAboutEquals(Vector3D a, Vector3D b, double tolerance) {
        return a.distance(b) <= tolerance;
    }
}
