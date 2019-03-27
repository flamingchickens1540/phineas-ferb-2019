package org.team1540.robot2019.vision;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class SimilarVector3DTracker {

    private Vector3D lastVector;
    private double maxDistance;

    public SimilarVector3DTracker(double maxDistance) {
        this.maxDistance = maxDistance;
    }

    public boolean isSimilarTransform(Vector3D newTransform) {
        if (lastVector == null) {
            this.lastVector = newTransform;
            return true;
        }
        double distance = lastVector.distance(newTransform);
        SmartDashboard.putNumber("Debug/SimilarVector3DTracker/Distance", distance); // todo: debug
        if (distance < maxDistance) {
            this.lastVector = newTransform;
            return true;
        }
        return false;
    }

    public Vector3D getVector3D() {
        return lastVector;
    }

    public void setVector3D(Vector3D lastTransform) {
        this.lastVector = lastTransform;
    }

    public void reset() {
        lastVector = null;
    }
}
