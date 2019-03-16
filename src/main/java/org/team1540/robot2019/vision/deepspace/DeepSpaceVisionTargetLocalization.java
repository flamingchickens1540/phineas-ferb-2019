package org.team1540.robot2019.vision.deepspace;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.vision.DualVisionTargetLocalizationUtils;

public class DeepSpaceVisionTargetLocalization {

    private double planeHeight;
    private final Consumer<Transform3D> onUpdate;
    private final DeepSpaceVisionTargetCamera camera;
    private Transform3D baseLinkToVisionTarget;
    private long timeLastAcquired = 0;

    public DeepSpaceVisionTargetLocalization(DeepSpaceVisionTargetCamera camera, double planeHeight, double updatePeriod, Consumer<Transform3D> onUpdate) {
        this.camera = camera;
        this.onUpdate = onUpdate;
        this.planeHeight = planeHeight;
        new Notifier(this::attemptUpdatePose).startPeriodic(updatePeriod);
    }

    public boolean attemptUpdatePose() {
        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull();

        SmartDashboard.putBoolean("DeepSpaceVisionTargetLocalization/Debug/HasFound", rawVisionTarget != null);
        if (rawVisionTarget == null) {
            return false;
        }
        timeLastAcquired = System.currentTimeMillis();

        baseLinkToVisionTarget =
            DualVisionTargetLocalizationUtils.poseFromTwoRawCamPoints(
                rawVisionTarget.getPointA(),
                rawVisionTarget.getPointB(),
                planeHeight,
                camera.getBaseLinkToCamera().getPosition(),
                camera.getBaseLinkToCamera().getOrientation()
            );

        onUpdate.accept(baseLinkToVisionTarget);
        return true;
    }

    public void setPlaneHeight(double planeHeight) {
        this.planeHeight = planeHeight;
    }

    public boolean targetWasAcquired() {
        return baseLinkToVisionTarget != null;
    }

    public Transform3D getLastBaseLinkToVisionTarget() {
        return baseLinkToVisionTarget;
    }

    public long millisSinceLastAcquired() {
        return System.currentTimeMillis() - timeLastAcquired;
    }
}
