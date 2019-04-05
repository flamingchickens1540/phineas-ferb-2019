package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.vision.SimilarVector3DTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;

public class PercentManualLineupLocalizationAngleProvider implements PointAngleProvider {

    private static final Logger logger = Logger.getLogger(PercentManualLineupLocalizationAngleProvider.class);
    private static final double MAX_ACCURATE_POSE_DISTANCE = 1.3;
    private static double HATCH_GRAB_X_OFFSET = -0.05;
    private static double HATCH_GRAB_Y_OFFSET = 0.01;
    private static double HATCH_PLACE_X_OFFSET = -0.1;
    private static double HATCH_PLACE_Y_OFFSET = 0.01;
    private static double M = 1.2;
    private static double Z = 0.6;
    private static double A = 2;

    private static double POINT_DEADZONE = 0.6;

    private static double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular VPID controller
    private static double P = 0.45;
    private static double I = 0;
    private static double D = 1;

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity


    private Transform3D goal;
    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;

    private final SimilarVector3DTracker similarVectorTracker = new SimilarVector3DTracker(0.4);
    private Timer timer;

    public PercentManualLineupLocalizationAngleProvider(TankDriveOdometryAccumulatorRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization) {
//        super(P, I, D, OUTPUT_SCALAR, MAX, MIN, DEADZONE, THROTTLE_CONSTANT);
//        requires(Robot.drivetrain);

        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;

        SmartDashboard.putNumber("PercentLineupLocalization/OUTPUT_SCALAR", OUTPUT_SCALAR);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KP", P);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KI", I);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KD", D);
        SmartDashboard.putNumber("PercentLineupLocalization/MAX_VEL_THETA", MAX);
        SmartDashboard.putNumber("PercentLineupLocalization/MIN_VEL_THETA", MIN);
        SmartDashboard.putNumber("PercentLineupLocalization/DEADZONE_VEL_THETA", DEADZONE);
//        SmartDashboard.putNumber("PercentLineupLocalization/ANGLE_OFFSET", ANGLE_OFFSET);
        SmartDashboard.putNumber("PercentLineupLocalization/HATCH_GRAB_X_OFFSET", HATCH_GRAB_X_OFFSET);
        SmartDashboard.putNumber("PercentLineupLocalization/HATCH_GRAB_Y_OFFSET", HATCH_GRAB_Y_OFFSET);
        SmartDashboard.putNumber("PercentLineupLocalization/HATCH_PLACE_X_OFFSET", HATCH_PLACE_X_OFFSET);
        SmartDashboard.putNumber("PercentLineupLocalization/HATCH_PLACE_Y_OFFSET", HATCH_PLACE_Y_OFFSET);
        SmartDashboard.putNumber("PercentLineupLocalization/A", A);
        SmartDashboard.putNumber("PercentLineupLocalization/M", M);
        SmartDashboard.putNumber("PercentLineupLocalization/Z", Z);
        SmartDashboard.putNumber("PercentLineupLocalization/POINT_DEADZONE", POINT_DEADZONE);

        enableHatchModeForNextCycle();
    }


    public void justLetGoReset() {
        if (timer == null) {
            timer = new Timer();
        }
        timer.reset();
        similarVectorTracker.reset();
    }

    public void pointNextReset() {
        timer = null;
        similarVectorTracker.reset();
    }

    @Override
    public void initialize() {
        P = SmartDashboard.getNumber("PercentLineupLocalization/OUTPUT_SCALAR", OUTPUT_SCALAR);
        P = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KP", P);
        I = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KI", I);
        D = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KD", D);
        MAX = SmartDashboard.getNumber("PercentLineupLocalization/MAX_VEL_THETA", MAX);
        MIN = SmartDashboard.getNumber("PercentLineupLocalization/MIN_VEL_THETA", MIN);
        DEADZONE = SmartDashboard.getNumber("PercentLineupLocalization/DEADZONE_VEL_THETA", DEADZONE);
        HATCH_GRAB_X_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_GRAB_X_OFFSET", HATCH_GRAB_X_OFFSET);
        HATCH_GRAB_Y_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_GRAB_Y_OFFSET", HATCH_GRAB_Y_OFFSET);
        HATCH_PLACE_X_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_PLACE_X_OFFSET", HATCH_PLACE_X_OFFSET);
        HATCH_PLACE_Y_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_PLACE_Y_OFFSET", HATCH_PLACE_Y_OFFSET);
        A = SmartDashboard.getNumber("PercentLineupLocalization/A", A);
        M = SmartDashboard.getNumber("PercentLineupLocalization/M", M);
        Z = SmartDashboard.getNumber("PercentLineupLocalization/Z", Z);
        POINT_DEADZONE = SmartDashboard.getNumber("PercentLineupLocalization/POINT_DEADZONE", POINT_DEADZONE);

        goal = null;

        Hardware.limelight.prepForVision();

        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    private void enableHatchModeForNextCycle() {
        logger.debug("Hatch mode!");
        long pipeline = Hardware.limelight.getPipeline();
        if (pipeline != 2 && pipeline != 3) {
            Hardware.limelight.setPipeline(0);
        }
        Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.HATCH_TARGET_HEIGHT);
    }

    public void enableRocketBallModeForNextCycle() {
        logger.debug("Rocket ball mode!");
        Hardware.limelight.setPipeline(1);
        Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.ROCKET_BALL_TARGET_HEIGHT);
        similarVectorTracker.reset();
    }

    @Override
    public PointControlConfig getPointControlConfig() {
        return new PointControlConfig(OUTPUT_SCALAR, MIN, MAX, DEADZONE, P, I, D, THROTTLE_CONSTANT);
    }

    @Override
    public double returnAngleError(double defaultError) { // TODO: If speed is too large, reset similar vector tracking
        if (deepSpaceVisionTargetLocalization.attemptUpdatePose() && Robot.elevator.getPosition() < 3) { // TODO: This should be a tuning constant
            Transform3D goal = computeGoal();
//            if (timer != null && !timer.hasPeriodPassed(0)) {
//                similarVectorTracker.reset();
//            }
            if (deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget().toTransform2D().getPositionVector().distance(Vector2D.ZERO) > MAX_ACCURATE_POSE_DISTANCE) {
                similarVectorTracker.setVector3D(goal.getPosition());
                this.goal = goal;
            }
            if (similarVectorTracker.isSimilarTransform(goal.getPosition())) {
                this.goal = goal;
            } else {
                logger.debug("Ignoring pose estimate- varies by more than the tolerance!");
            }
        }

        if (goal != null) {
            return getAngleError();
        } else {
            return defaultError;
        }
    }

    public boolean hasGoalBeenFound() {
        return goal != null;
    }

    private Transform3D computeGoal() {
        Transform3D partialGoal = driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget());
        if (Robot.hatch.isRetracted()) {
            partialGoal = partialGoal.add(new Transform3D(HATCH_PLACE_X_OFFSET, HATCH_PLACE_Y_OFFSET, 0));
        } else {
            partialGoal = partialGoal.add(new Transform3D(HATCH_GRAB_X_OFFSET, HATCH_GRAB_Y_OFFSET, 0));
        }
        return partialGoal;
    }


    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
//        double x = -(M * smoothStep(1.0 / (A - Z) * (distanceToVisionTarget - Z)));
//        double x1 = -0.15 * smoothStepFiveDir(distanceToVisionTarget - 0.5);
//        SmartDashboard.putNumber("DistanceOffset", x1);
//        Transform3D adjustedGoal = goal.add(new Transform3D(x1, 0, 0));
//        if (distanceToVisionTarget < POINT_DEADZONE) {
//            adjustedGoal = goal;
//        }
//        Vector3D goalPosition = adjustedGoal.getPosition();
        Vector3D goalPosition = goal.getPosition();
        double targetAngle = Math.atan2(goalPosition.getY() - odomPosition.getY(), goalPosition.getX() - odomPosition.getX());
        double signedAngleError = TrigUtils.signedAngleError(targetAngle, Hardware.navx.getYawRadians());
        double xVel = Robot.drivetrain.getTwist().getX();
        if (Math.abs(xVel) > 1.4) {
            logger.debug("xVel greater than 1.4, resetting similar pose tracker");
            this.pointNextReset();
            return 0;
        }
        if (Math.abs(signedAngleError) > Math.PI / 2) {
            logger.debug("Error is greater than PI/2, resetting similar pose tracker");
            this.pointNextReset();
            return 0;
        }
        return signedAngleError;
    }

    public double getDistanceToVisionTarget() {
        if (goal == null) {
            return Double.POSITIVE_INFINITY;
        }
        double distanceToVisionTarget = driveOdometry.getOdomToBaseLink().toTransform2D().getPositionVector().distance(goal.toTransform2D().getPositionVector());
        SmartDashboard.putNumber("DistanceToTarget", distanceToVisionTarget);
        return distanceToVisionTarget;
    }

    //
//    private static double smoothStep(double x) {
//        if (x > 1) {
//            return 1;
//        } else if (x < 0) {
//            return 0;
//        }
//        return 3 * x * x - 2 * x * x * x;
//    }
//
//    private static double smoothStepFiveDir(double x) {
//        if (x > 1) {
//            return 1;
//        } else if (x < 0) {
//            return 0;
//        }
//        return 30 * x * x * x * x - 60 * x * x * x + 30 * x * x;
//    }
//
    public void end() {
        enableHatchModeForNextCycle();
    }
}
