package org.team1540.robot2019.vision.deepspace;

import javax.annotation.Nullable;
import org.team1540.robot2019.datastructures.threed.Transform3D;

public interface DeepSpaceVisionTargetCamera {

    double getHorizontalFov();

    double getVerticalFov();

    Transform3D getBaseLinkToCamera();

    void setBaseLinkToCamera(Transform3D baseLinkToCamera);

    @Nullable
    RawDeepSpaceVisionTarget getRawDeepSpaceVisionTargetOrNull();
}
