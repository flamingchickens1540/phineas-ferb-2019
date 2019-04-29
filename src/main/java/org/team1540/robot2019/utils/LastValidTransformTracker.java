package org.team1540.robot2019.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.team1540.robot2019.datastructures.threed.Transform3D;

// TODO: This temporary class will be replaced by TF
public class LastValidTransformTracker implements Consumer<Transform3D> {

    private Transform3D transform3D;
    private Supplier<Transform3D> transform3DSupplier;

    public LastValidTransformTracker(Supplier<Transform3D> transform3DSupplier) {
        this.transform3DSupplier = transform3DSupplier;
    }

    @Override
    public void accept(Transform3D baseLinkToVisionTarget) {
        transform3D = transform3DSupplier.get().add(baseLinkToVisionTarget);
    }

    public Transform3D getTransform3D() {
        return transform3D;
    }
}
