package org.team1540.robot2019.vision;

import edu.wpi.first.wpilibj.Notifier;
import java.util.function.Supplier;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.wrappers.Limelight;

/**
 * Class for localization with a Limelight over NetworkTables.
 */
public class LimelightLocalization {

    private static final Vector3D CAMERA_POSITION = new Vector3D(0.086, 0.099, 1.12);
    private static final double PLANE_HEIGHT = 0.71;
    private static final double CAMERA_ROLL = Math.toRadians(0);
    private static final double CAMERA_TILT = Math.toRadians(-37.5);

    private Supplier<Transform3D> odomSupplier = null;

    private Transform3D odomToVisionTarget;
    private Transform3D baseLinkToVisionTarget;
    private Limelight limelight;
    private long timeLastAcquired = 0;

    public LimelightLocalization(Limelight limelight) {
        this.limelight = limelight;
    }

    public LimelightLocalization(Limelight limelight, double updatePeriod) {
        this(limelight);
        new Notifier(this::attemptUpdatePose).startPeriodic(updatePeriod);
    }

    public LimelightLocalization(Limelight limelight, double updatePeriod, Supplier<Transform3D> odomSupplier) {
        this(limelight);
        this.odomSupplier = odomSupplier;
        new Notifier(this::attemptUpdatePose).startPeriodic(updatePeriod);
    }

    public boolean attemptUpdatePose() {

        // TODO: Filter limelight contours using size, angle, etc.

        double upperLimit = 0.86;
//      double lowerLimit = 0.29; // With U
        double lowerLimit = -0.65;
        double leftAndRightLimit = 0.90;

        Vector2D point0 = limelight.getRawContour(0);
        Vector2D point1 = limelight.getRawContour(1);

        if (!limelight.isTargetFound()
            || !VisionUtils.isWithinBounds(point0, upperLimit, lowerLimit, leftAndRightLimit, -leftAndRightLimit)
            || !VisionUtils.isWithinBounds(point1, upperLimit, lowerLimit, leftAndRightLimit, -leftAndRightLimit)) {
            return false;
        }

        Rotation cameraTilt = new Rotation(Vector3D.PLUS_J, CAMERA_TILT, RotationConvention.FRAME_TRANSFORM);
        Rotation cameraRoll = new Rotation(Vector3D.PLUS_I, CAMERA_ROLL, RotationConvention.FRAME_TRANSFORM);

//        Rotation cameraRotation = new Rotation(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM, CAMERA_ROLL, CAMERA_TILT, 0);
        Rotation cameraRotation = cameraTilt.applyTo(cameraRoll);
        baseLinkToVisionTarget = DualVisionTargetLocalizationUtils
            .poseFromTwoCamPoints(point0, point1, PLANE_HEIGHT, CAMERA_POSITION, cameraRotation, Limelight.HORIZONTAL_FOV, Limelight.VERTICAL_FOV);

        if (odomSupplier != null) {
            odomToVisionTarget = odomSupplier.get().add(baseLinkToVisionTarget);
        }

        timeLastAcquired = System.currentTimeMillis();
        return true;
    }

    public boolean targetWasAcquired() {
        return baseLinkToVisionTarget != null;
    }

    public Transform3D getLastBaseLinkToVisionTarget() {
        return baseLinkToVisionTarget;
    }

    public Transform3D getLastOdomToVisionTarget() {
        return odomToVisionTarget;
    }

    public long millisSinceLastAcquired() {
        return System.currentTimeMillis() - timeLastAcquired;
    }
}
