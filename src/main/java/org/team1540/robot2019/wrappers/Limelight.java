package org.team1540.robot2019.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.UnitsUtils;
import org.team1540.robot2019.vision.DualVisionTargetLocalizationUtils;
import org.team1540.robot2019.vision.VisionUtils;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetCamera;
import org.team1540.robot2019.vision.deepspace.RawDeepSpaceVisionTarget;

public class Limelight implements DeepSpaceVisionTargetCamera {

    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(45.7);

    private final NetworkTable limelightTable;
    private Transform3D baseLinkToCamera;

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     * @param baseLinkToCamera baseLinkToCamera of the limelight
     */
    public Limelight(String name, Transform3D baseLinkToCamera) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        this.baseLinkToCamera = baseLinkToCamera;

        setPipeline(0);
    }

    @Override
    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }

    @Override
    public double getVerticalFov() {
        return VERTICAL_FOV;
    }

    @Override
    public Transform3D getBaseLinkToCamera() {
        return baseLinkToCamera;
    }

    @Override
    public void setBaseLinkToCamera(Transform3D baseLinkToCamera) {
        this.baseLinkToCamera = baseLinkToCamera;
    }

    /**
     * Gets the output of the limelight targeting from the network table.
     *
     * @return a {@link Vector2D} containing the output angles of the limelight targeting in radians
     */
    public Vector2D getTargetAngles() { // TODO: This should be negated appropriately
        double x = Math.toRadians(limelightTable.getEntry("tx").getDouble(0));
        double y = Math.toRadians(limelightTable.getEntry("ty").getDouble(0));
        return new Vector2D(x, y);
    }

    /**
     * Queries whether the limelight target has been found.
     *
     * @return the state of the target
     */
    public boolean isTargetFound() {
        return (double) limelightTable.getEntry("tv").getNumber(0) > 0;
    }

    /**
     * Gets additional raw contour centers published by the limelight. SendRawContours in the limelight web interface must be turned on.
     *
     * @param id the index of the raw contour
     * @return a {@link Vector2D} containing the center of the contour in screen-space coordinates
     */
    public Vector2D getRawContour(int id) {
        return new Vector2D(
            -limelightTable.getEntry("tx" + id).getDouble(0), // TODO: X should not be negated here
            limelightTable.getEntry("ty" + id).getDouble(0)
        );
    }

    /**
     * Gets additional raw contour centers published by the limelight. SendRawContours in the limelight web interface must be turned on.
     *
     * @param id the index of the raw contour
     * @return a {@link Vector2D} containing the center of the contour in screen-space coordinates or null if the contour does not pass the filters
     */
    public Vector2D getFilteredRawContourOrNull(int id) {
        double upperLimit = 0.86;
//      double lowerLimit = 0.29; // With U
        double lowerLimit = -0.65;
        double rightLimit = 0.90;
        double leftLimit = -0.90;
        Vector2D vector2D = getRawContour(id);
        if (vector2D.equals(Vector2D.ZERO)
            || !VisionUtils.isWithinBounds(vector2D, upperLimit, lowerLimit, rightLimit, leftLimit)) {
            return null;
        }
        return vector2D;
    }

    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param isOn the new state of the LEDs
     */
    public void setLeds(boolean isOn) {
        limelightTable.getEntry("ledMode").setNumber(isOn ? 0 : 1);
        NetworkTableInstance.getDefault().flush();
    }

    public boolean getLeds() {
        return limelightTable.getEntry("ledMode").getDouble(1) == 0;
    }

    /**
     * Sets limelight to driver cam or vision mode.
     *
     * @param driverCam Whether the limelight should be in driver cam mode
     */
    public void setDriverCam(boolean driverCam) {
        limelightTable.getEntry("camMode").setNumber(driverCam ? 1 : 0);
        NetworkTableInstance.getDefault().flush();
    }

    public void setPipeline(int id) {
        limelightTable.getEntry("pipeline").setNumber(id);
        NetworkTableInstance.getDefault().flush();
    }

    // todo: getPipeline and incorrect pipline warnings

    public void prepForVision() {
        setLeds(true);
        setDriverCam(false);
    }

    public void prepForDriverCam() {
        if (SmartDashboard.getBoolean("TurnOffLimelightWhenNotInUse", false)) {
            setLeds(false);
        }
//        setDriverCam(true);
    }

    /**
     * Attempts to get the published SolvePNP transform from the vision target to the limelight (not the other way around).
     *
     * @return the transform from the vision target to the limelight
     */
    public Transform3D getVisionTargetToLimelightTransformOrNull() {
        Double[] rawTransformation = limelightTable.getEntry("camtran").getDoubleArray(new Double[]{});
        if (rawTransformation[2] == 0) {
            return null;
        }
        // TODO: Something about this is probably wrong
        return new Transform3D(
            UnitsUtils.inchesToMeters(rawTransformation[2]),
            UnitsUtils.inchesToMeters(-rawTransformation[0]),
            UnitsUtils.inchesToMeters(-rawTransformation[1]),
            -Math.toRadians(rawTransformation[5]),
            Math.toRadians(rawTransformation[3]),
            Math.toRadians(rawTransformation[4]));
    }

    public List<Vector2D> getCorners() {
        Double[] xCorners = limelightTable.getEntry("tcornx").getDoubleArray(new Double[]{});
        Double[] yCorners = limelightTable.getEntry("tcorny").getDoubleArray(new Double[]{});
        List<Vector2D> cornerList = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            cornerList.add(new Vector2D(xCorners[i], yCorners[i]));
        }
        return cornerList;
    }

    @Override
    public RawDeepSpaceVisionTarget getRawDeepSpaceVisionTargetOrNull() {
        if (!isTargetFound()) {
            return null;
        }

        List<Vector2D> corners = getCorners();
        corners.sort(Comparator.comparingDouble(Vector2D::getX));
        double approxMiddleX = corners.get(0).getX()+corners.get(corners.size()-1).getX();
        Optional<Vector2D> min = corners.stream().filter(item -> item.getX() < approxMiddleX).min(Comparator.comparingDouble(Vector2D::getY));
        Optional<Vector2D> max = corners.stream().filter(item -> item.getX() > approxMiddleX).min(Comparator.comparingDouble(Vector2D::getY));

        if (!min.isPresent() || !max.isPresent()) {
            return null;
        }

        // TODO: Angles or normalized screen space?
        return new RawDeepSpaceVisionTarget(
            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(min.get(), getHorizontalFov(), getVerticalFov()),
            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(max.get(), getHorizontalFov(), getVerticalFov())
        );

//        // single point approach
//        return new RawDeepSpaceVisionTarget(this.getTargetAngles(), this.getTargetAngles());
//
//        // raw contours approach
//        Vector2D point0 = getFilteredRawContourOrNull(0);
//        Vector2D point1 = getFilteredRawContourOrNull(1);
//
//        if (point0 == null || point1 == null) {
//            return null;
//        }
//
//        return new RawDeepSpaceVisionTarget(
//            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(point0, getHorizontalFov(), getVerticalFov()),
//            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(point1, getHorizontalFov(), getVerticalFov())
//        );
    }

    public void setPipeline(double pipelineID) { // TODO
        limelightTable.getEntry("pipeline").setDouble(1);
    }
}
