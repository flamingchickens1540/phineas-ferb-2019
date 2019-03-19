package org.team1540.robot2019.vision.deepspace;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;
import java.util.function.DoubleUnaryOperator;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.RotationUtils;
import org.team1540.robot2019.vision.DualVisionTargetLocalizationUtils;

public class DeepSpaceVisionTargetLocalization {

    private static final Logger logger = Logger.getLogger(DeepSpaceVisionTargetLocalization.class);

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

    public Double estimateCorrectPitch(double actualDistance, int maxAttempts, double tolerance) {

        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull();

        if (rawVisionTarget == null) {
            logger.error("No vision target found!");
            return null;
        }

        double[] angles = camera.getBaseLinkToCamera().getOrientation().getAngles(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM);

        double startPitch = angles[1];

        DoubleUnaryOperator function = (attemptPitch) -> {
            baseLinkToVisionTarget =
                DualVisionTargetLocalizationUtils.poseFromTwoRawCamPoints(
                    rawVisionTarget.getPointA(),
                    rawVisionTarget.getPointB(),
                    planeHeight,
                    camera.getBaseLinkToCamera().getPosition(),
                    RotationUtils.fromRPY(angles[0], attemptPitch, angles[2])
                );
            return baseLinkToVisionTarget.toTransform2D().getPositionVector().distance(Vector2D.ZERO);
        };

        Double result = doubleBinarySearch(startPitch, Math.toRadians(10), actualDistance, tolerance, maxAttempts, function);

        if (result == null) {
            logger.error("Unable to find good pitch angle");
        }
        return result;
    }


    public Double estimateCorrectYaw(double actualDistance, int maxAttempts, double tolerance) {

        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull();

        if (rawVisionTarget == null) {
            logger.error("No vision target found!");
            return null;
        }

        double[] angles = camera.getBaseLinkToCamera().getOrientation().getAngles(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM);

        double startYaw = angles[2];

        DoubleUnaryOperator function = (attemptYaw) -> {
            baseLinkToVisionTarget =
                DualVisionTargetLocalizationUtils.poseFromTwoRawCamPoints(
                    rawVisionTarget.getPointA(),
                    rawVisionTarget.getPointB(),
                    planeHeight,
                    camera.getBaseLinkToCamera().getPosition(),
                    RotationUtils.fromRPY(angles[0], angles[1], attemptYaw)
                );
            return baseLinkToVisionTarget.toTransform2D().getPositionVector().getY();
        };

        Double result = doubleBinarySearch(startYaw, Math.toRadians(10), actualDistance, tolerance, maxAttempts, function);

        if (result == null) {
            logger.error("Unable to find good yaw angle");
        }
        return result;
    }


    public static Double doubleBinarySearch(double inputGuess, double inputMargin, double actualOutput, double outputTolerance, int maxAttempts, DoubleUnaryOperator method) {
        double upperOutput = method.applyAsDouble(inputGuess + inputMargin);
        double lowerOutput = method.applyAsDouble(inputGuess - inputMargin);
        boolean direction = upperOutput > lowerOutput;
        if (direction) {
            if (actualOutput < lowerOutput || actualOutput > upperOutput) {
                logger.error("Search failed! Actual outside of guess bounds!");
                return null;
            }
        } else {
            if (actualOutput > lowerOutput || actualOutput < upperOutput) {
                logger.error("Search failed! Actual outside of guess bounds!");
                return null;
            }
        }
        double upperInput = inputGuess + inputMargin;
        double lowerInput = inputGuess - inputMargin;

        int attemptCounter = 0;
        while (attemptCounter < maxAttempts) {
            attemptCounter++;
            double avgInput = (upperInput + lowerInput) / 2;
            double avgOutput = method.applyAsDouble(avgInput);
            if (Math.abs(avgOutput - actualOutput) < outputTolerance) {
                return avgInput;
            } else {
                if (avgOutput > actualOutput ^ !direction) {
                    upperInput = avgInput;
                } else {
                    lowerInput = avgInput;
                }
            }
        }
        logger.error("Search failed! Unable to find guess within tolerance");
        return null;
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
