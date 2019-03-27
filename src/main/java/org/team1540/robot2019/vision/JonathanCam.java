package org.team1540.robot2019.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import javax.annotation.Nullable;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetCamera;
import org.team1540.robot2019.vision.deepspace.RawDeepSpaceVisionTarget;

public class JonathanCam implements DeepSpaceVisionTargetCamera {

    private static final Logger logger = Logger.getLogger(JonathanCam.class);

    private final NetworkTable picamTable;
    private Transform3D baseLinkToCamera;

    /**
     * Constructs a new JonathanCam interface with the default hostname.
     *
     * @param name hostname of the JonathanCam
     * @param baseLinkToCamera baseLinkToCamera of the JonathanCam
     */
    public JonathanCam(String name, Transform3D baseLinkToCamera) {
        picamTable = NetworkTableInstance.getDefault().getTable(name);
        this.baseLinkToCamera = baseLinkToCamera;
    }

    /**
     * Constructs a new JonathanCam interface with the default hostname.
     *
     * @param baseLinkToCamera baseLinkToCamera of the JonathanCam
     */
    public JonathanCam(Transform3D baseLinkToCamera) {
        this("raspi-2", baseLinkToCamera);
    }

    @Override
    public Transform3D getBaseLinkToCamera() {
        return null;
    }

    @Override
    public boolean isTargetFound() {
        return picamTable.getEntry("vectors").getDoubleArray(new double[]{}).length == 6;
    }

    @Nullable
    @Override
    public RawDeepSpaceVisionTarget getRawDeepSpaceVisionTargetOrNull() {
        double[] vectors = picamTable.getEntry("vectors").getDoubleArray(new double[]{});

        if (vectors.length != 6) {
            return null;
        }

        Vector3D vectorA = new Vector3D(
            vectors[0],
            vectors[1],
            vectors[2]
        );

        Vector3D vectorB = new Vector3D(
            vectors[0],
            vectors[1],
            vectors[2]
        );

        Vector2D anglesA = new Vector2D(
            Math.atan2(vectorA.getX(), vectorA.getZ()),
            Math.atan2(vectorA.getY(), vectorA.getZ())
        );

        Vector2D anglesB = new Vector2D(
            Math.atan2(vectorB.getX(), vectorB.getZ()),
            Math.atan2(vectorB.getY(), vectorB.getZ())
        );

        logger.debug(String.format("A: %f %f B: %f %f",
            Math.toDegrees(anglesA.getX()),
            Math.toDegrees(anglesA.getY()),
            Math.toDegrees(anglesB.getX()),
            Math.toDegrees(anglesB.getY())
        ));

        return new RawDeepSpaceVisionTarget(anglesA, anglesB);
    }
}
