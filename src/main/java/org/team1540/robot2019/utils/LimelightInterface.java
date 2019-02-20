package org.team1540.robot2019.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.datastructures.threed.Transform3D;

public class LimelightInterface {

    private final NetworkTable limelightTable;

    /**
     * Constructs a new limelight interface with hostname {@code name}.
     *
     * @param name hostname of the new limelight
     */
    public LimelightInterface(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * Constructs a new limelight interface with the default hostname.
     */
    public LimelightInterface() {
        this("limelight");
    }

    /**
     * Gets the output of the limelight targeting from the network table.
     *
     * @return a {@link Vector2D} containing the output angles of the limelight targeting in radians
     */
    public Vector2D getTargetAngles() {
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
     * Sets limelight's green LEDs on or off.
     *
     * @param isOn the new state of the LEDs
     */
    public void setLeds(boolean isOn) {
        limelightTable.getEntry("ledMode").setNumber(isOn ? 0 : 1);
        NetworkTableInstance.getDefault().flush();
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
}
