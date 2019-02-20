package org.team1540.robot2019.vision;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;

/**
 * Class for localization with a Limelight over NetworkTables.
 */
public class LimelightLocalization {

    private Transform3D baseLinkToVisionTarget;
    private LimelightInterface limelightInterface;
    private long timeLastAcquired = 0;

    public LimelightLocalization(String limeTableName) {
        limelightInterface = new LimelightInterface(limeTableName);
    }

    public boolean attemptUpdatePose() {
        double CAMERA_TILT = Math.toRadians(-37.5);
        double CAMERA_ROLL = Math.toRadians(0);
        double PLANE_HEIGHT = 0.71; // Height of vision targets in meters
        Vector3D CAMERA_POSITION = new Vector3D(0.086, 0.099, 1.12); // Position of camera in meters

        // TODO: Filter limelight contours using size, angle, etc.

        double upperLimit = 0.86;
//      double lowerLimit = 0.29; // With U
        double lowerLimit = -0.65;
        double leftAndRightLimit = 0.90;

        Vector2D point0 = limelightInterface.getRawContour(0);
        Vector2D point1 = limelightInterface.getRawContour(1);

        if (!limelightInterface.isTargetFound()
            || !VisionUtils.isWithinBounds(point0, upperLimit, lowerLimit, leftAndRightLimit, -leftAndRightLimit)
            || !VisionUtils.isWithinBounds(point1, upperLimit, lowerLimit, leftAndRightLimit, -leftAndRightLimit)) {
            return false;
        }

        Rotation cameraTilt = new Rotation(Vector3D.PLUS_J, CAMERA_TILT, RotationConvention.FRAME_TRANSFORM);
        Rotation cameraRoll = new Rotation(Vector3D.PLUS_I, CAMERA_ROLL, RotationConvention.FRAME_TRANSFORM);

        Rotation cameraRotation = new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, CAMERA_ROLL, CAMERA_TILT, 0);
//        Rotation cameraRotation = cameraTilt.applyTo(cameraRoll);
        baseLinkToVisionTarget = DualVisionTargetLocalizationUtils
            .poseFromTwoCamPoints(point0, point1, PLANE_HEIGHT, CAMERA_POSITION, cameraRotation, Tuning.LIMELIGHT_HORIZONTAL_FOV, Tuning.LIMELIGHT_VERTICAL_FOV);

        timeLastAcquired = System.currentTimeMillis();
        return true;
    }

    public boolean targetWasAcquired() {
        return baseLinkToVisionTarget != null;
    }

    public Transform3D getBaseLinkToVisionTarget() {
        return baseLinkToVisionTarget;
    }

    public long millisSinceLastAcquired() {
        return System.currentTimeMillis() - timeLastAcquired;
    }
}
