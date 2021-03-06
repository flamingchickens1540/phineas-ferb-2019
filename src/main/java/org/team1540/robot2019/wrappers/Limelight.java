package org.team1540.robot2019.wrappers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.UnitsUtils;
import org.team1540.robot2019.vision.DualVisionTargetLocalizationUtils;
import org.team1540.robot2019.vision.RawContour;
import org.team1540.robot2019.vision.VisionUtils;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetCamera;
import org.team1540.robot2019.vision.deepspace.RawDeepSpaceVisionTarget;

public class Limelight implements DeepSpaceVisionTargetCamera {

    private static final Logger logger = Logger.getLogger(Limelight.class);

    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(45.7);
    private static final Vector2D CAM_RESOLUTION = new Vector2D(320, 240);

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
    }

    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }

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
    public RawContour getRawContour(int id) {
        return new RawContour(id, new Vector2D(
            -limelightTable.getEntry("tx" + id).getDouble(0), // TODO: X should not be negated here
            limelightTable.getEntry("ty" + id).getDouble(0)
        ));
    }

    /**
     * Gets additional raw contour centers published by the limelight. SendRawContours in the limelight web interface must be turned on.
     *
     * @param id the index of the raw contour
     * @return a {@link Vector2D} containing the center of the contour in screen-space coordinates or null if the contour does not pass the filters
     */
    public RawContour getFilteredRawContourOrNull(int id) {
        double upperLimit = 1;
//      double lowerLimit = 0.29; // With U
        double lowerLimit = -0.65;
        double rightLimit = 0.90;
        double leftLimit = -0.90;
        RawContour contour = getRawContour(id);
        if (contour.getCenter().equals(Vector2D.ZERO)
            || !VisionUtils.isWithinBounds(contour.getCenter(), upperLimit, lowerLimit, rightLimit, leftLimit)) {
            return null;
        }
        return contour;
    }

    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param isOn the new state of the LEDs
     */
    public void setLeds(boolean isOn) {
        if (getLeds() != isOn) {
            limelightTable.getEntry("ledMode").setNumber(isOn ? 0 : 1);
            NetworkTableInstance.getDefault().flush();
        }
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

    public void setPipeline(double id) {
        if (getPipeline() != id) {
            limelightTable.getEntry("pipeline").setNumber(id);
            NetworkTableInstance.getDefault().flush();
        }
    }

    public long getPipeline() {
        return Math.round((double) limelightTable.getEntry("getpipe").getNumber(-1));
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
        // TODO: These negations might be incorrect
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

        if (limelightTable.getEntry("ta").getDouble(0) < 0.47) { // TODO: Make this tunable
            return null;
        }

        // raw contours approach
        RawContour[] rawContours = new RawContour[]{
            getFilteredRawContourOrNull(0),
            getFilteredRawContourOrNull(1),
            getFilteredRawContourOrNull(2)
        };
        List<RawContour> sortedContours = Arrays.stream(rawContours).filter(Objects::nonNull)
            .map(point -> new RawContour(point.getId(), DualVisionTargetLocalizationUtils.anglesFromScreenSpace(point.getCenter(), getHorizontalFov(), getVerticalFov())))
            .sorted(Comparator.comparingDouble(point -> point.getCenter().distance(this.getTargetAngles()))).collect(Collectors.toList());

        if (sortedContours.size() < 2) { // TODO: ScreenspaceContour class and AnglesContour and VectorContour
            return null;
        }
        // TODO: Add pipeline checks

        return new RawDeepSpaceVisionTarget(
            sortedContours.get(0).getCenter(),
            sortedContours.get(1).getCenter(),
            getTargetAngles()
        );
    }
}
