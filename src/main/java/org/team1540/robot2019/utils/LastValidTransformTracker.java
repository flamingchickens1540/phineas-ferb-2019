package org.team1540.robot2019.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.team1540.robot2019.datastructures.threed.Transform3D;

// TODO: This temporary class will be replaced by TF
public class LastValidTransformTracker implements Consumer<Transform3D> {

    private Transform3D odomToVisionTarget;
    private Supplier<Transform3D> odomToBaseLinkSupplier;

    public LastValidTransformTracker(Supplier<Transform3D> odomToBaseLinkSupplier) {
        this.odomToBaseLinkSupplier = odomToBaseLinkSupplier;
    }

    @Override
    public void accept(Transform3D baseLinkToVisionTarget) {
        odomToVisionTarget = odomToBaseLinkSupplier.get().add(baseLinkToVisionTarget);
    }

    public Transform3D getOdomToVisionTarget() {
        return odomToVisionTarget;
    }
}
