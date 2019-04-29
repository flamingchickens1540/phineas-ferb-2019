package org.team1540.robot2019.vision;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class RawContour {

    private int id;
    private Vector2D center;

    public RawContour(int id, Vector2D center) {
        this.id = id;
        this.center = center;
    }

    public Vector2D getCenter() {
        return center;
    }

    public double getX() {
        return center.getX();
    }

    public double getY() {
        return center.getY();
    }

    public int getId() {
        return id;
    }
}
