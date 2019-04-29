package org.team1540.robot2019.vision.deepspace;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class RawDeepSpaceVisionTarget {

    private final Vector2D pointA;
    private final Vector2D pointB;
    private Vector2D centerPoint;

    public RawDeepSpaceVisionTarget(Vector2D pointA, Vector2D pointB) {
        this.pointA = pointA;
        this.pointB = pointB;
    }

    public RawDeepSpaceVisionTarget(Vector2D pointA, Vector2D pointB, Vector2D centerPoint) {
        this.pointA = pointA;
        this.pointB = pointB;
        this.centerPoint = centerPoint;
    }

    public Vector2D getPointA() {
        return pointA;
    }

    public Vector2D getPointB() {
        return pointB;
    }

    public Vector2D getCenterPoint() {
        return centerPoint;
    }
}
