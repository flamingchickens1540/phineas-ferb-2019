package org.team1540.robot2019.vision.deepspace;

import edu.wpi.first.wpilibj.Notifier;
import java.util.function.Consumer;
import java.util.function.DoubleUnaryOperator;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.RotationUtils;
import org.team1540.robot2019.vision.DualVisionTargetLocalizationUtils;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetCamera.TargetType;

public class DeepSpaceVisionTargetLocalization {

    private static final Logger logger = Logger.getLogger(DeepSpaceVisionTargetLocalization.class);

    private double planeHeight;
    private final TargetType type;
    private final Consumer<Transform3D> onUpdate;
    private final DeepSpaceVisionTargetCamera camera;
    private Transform3D baseLinkToVisionTarget;
    private long timeLastAcquired = 0;

    public DeepSpaceVisionTargetLocalization(DeepSpaceVisionTargetCamera camera, double planeHeight, TargetType type, double updatePeriod, Consumer<Transform3D> onUpdate) {
        this.camera = camera;
        this.type = type;
        this.onUpdate = onUpdate;
        this.planeHeight = planeHeight; // TODO: PlaceHeight should be part of TargetType
        new Notifier(this::attemptUpdatePose).startPeriodic(updatePeriod);
    }

    public boolean attemptUpdatePose() {
        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull(TargetType.HATCH_TARGET);

//        SmartDashboard.putBoolean("DeepSpaceVisionTargetLocalization/Debug/HasFound", rawVisionTarget != null);
        if (rawVisionTarget == null) {
            return false;
        }
        timeLastAcquired = System.currentTimeMillis();

        baseLinkToVisionTarget =
            DualVisionTargetLocalizationUtils.poseFromTwoRawCamPointsAndCenter(
                rawVisionTarget.getPointA(),
                rawVisionTarget.getPointB(),
                rawVisionTarget.getCenterPoint(),
                planeHeight,
                camera.getBaseLinkToCamera().getPosition(),
                camera.getBaseLinkToCamera().getOrientation()
            );

        if (baseLinkToVisionTarget.getPosition().getX() < 0) { // Target should not be behind the robot! (if "going to do a 180" -> "don't")
            logger.warn("Vision target found behind robot! This means the robot is detecting interference as targets. Ignoring pose estimate.");
            return false;
        }

        onUpdate.accept(baseLinkToVisionTarget);
        return true;
    }

    public Double estimateCorrectPitch(double actualDistance, int maxAttempts, double tolerance, boolean updateIfSuccessful) {
        if (this.type != TargetType.HATCH_TARGET) {
            logger.error("Calibration only supports hatch targets!");
            return null;
        }

        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull(type);

        if (rawVisionTarget == null) {
            logger.error("No vision target found!");
            return null;
        }

        double[] angles = camera.getBaseLinkToCamera().getOrientation().getAngles(RotationOrder.XYZ, RotationConvention.FRAME_TRANSFORM); // TODO: Use RotationUtils::getRPYVec

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

        Double result = doubleBinarySearch(startPitch, Math.toRadians(20), actualDistance, tolerance, maxAttempts, function);

        if (result == null) {
            logger.error("Unable to find good pitch angle");
        } else if (updateIfSuccessful) {
            camera.setBaseLinkToCamera(new Transform3D(camera.getBaseLinkToCamera().getPosition(), RotationUtils.fromRPY(angles[0], result, angles[2])));
        }
        return result;
    }


    public Double estimateCorrectYaw(double actualDistance, int maxAttempts, double tolerance, boolean updateIfSuccessful) {

        if (this.type != TargetType.HATCH_TARGET) {
            logger.error("Calibration only supports hatch targets!");
            return null;
        }
        RawDeepSpaceVisionTarget rawVisionTarget = camera.getRawDeepSpaceVisionTargetOrNull(type);

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
        } else if (updateIfSuccessful) {
            camera.setBaseLinkToCamera(new Transform3D(camera.getBaseLinkToCamera().getPosition(), RotationUtils.fromRPY(angles[0], angles[1], result)));
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
//
//    public void setPlaneHeight(double planeHeight) {
//        this.planeHeight = planeHeight;
//    }

    public boolean targetWasAcquired() {
        return baseLinkToVisionTarget != null;
    }

    public Transform3D getLastBaseLinkToVisionTarget() {
        return baseLinkToVisionTarget;
    }

    public long millisSinceLastAcquired() {
        return System.currentTimeMillis() - timeLastAcquired;
    }

    public TargetType getTargetType() {
        return type;
    }
}
