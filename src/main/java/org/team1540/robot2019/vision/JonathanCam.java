package org.team1540.robot2019.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import javax.annotation.Nullable;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetCamera;
import org.team1540.robot2019.vision.deepspace.RawDeepSpaceVisionTarget;

public class JonathanCam implements DeepSpaceVisionTargetCamera {

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

    @Nullable
    @Override
    public RawDeepSpaceVisionTarget getRawDeepSpaceVisionTargetOrNull() {
        double[] vectors = picamTable.getEntry("vectors").getDoubleArray(new double[]{});

        if (vectors.length != 6) {
            return null;
        }
        return null;
//        return new RawDeepSpaceVisionTarget(Math.atan2());
//
//        return new RawDeepSpaceVisionTarget(
//            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(point0, getHorizontalFov(), getVerticalFov()),
//            DualVisionTargetLocalizationUtils.anglesFromScreenSpace(point1, getHorizontalFov(), getVerticalFov())
//        );
    }
}
