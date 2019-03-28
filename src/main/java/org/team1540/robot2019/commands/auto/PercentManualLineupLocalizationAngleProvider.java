package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.vision.SimilarVector3DTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;

public class PercentManualLineupLocalizationAngleProvider implements PointAngleProvider {

    public static final Logger logger = Logger.getLogger(PercentManualLineupLocalizationAngleProvider.class);
    private static double HATCH_GRAB_X_OFFSET = -0.05;
    private static double HATCH_GRAB_Y_OFFSET = -0.015;
    private static double HATCH_PLACE_X_OFFSET = -0.05;
    private static double HATCH_PLACE_Y_OFFSET = -0.025;
    private static double A = 5.2;
    private static double B = 0;
    private static double C = 1.2;

    private static double POINT_DEADZONE = 0.6;

    private static double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular VPID controller
    private static double P = 0.39;
    private static double I = 0;
    private static double D = 0.6;

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity
//    public static double ANGLE_OFFSET = 0; // Degrees offset from center of target
//    public static double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target


    private Transform3D goal;
    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;

    private final SimilarVector3DTracker similarVectorTracker = new SimilarVector3DTracker(0.1);

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
        SmartDashboard.putNumber("PercentLineupLocalization/B", B);
        SmartDashboard.putNumber("PercentLineupLocalization/C", C);
        SmartDashboard.putNumber("PercentLineupLocalization/POINT_DEADZONE", POINT_DEADZONE);
    }


    @Override
    public void reset() {
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
//        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/ANGLE_OFFSET", ANGLE_OFFSET);
        HATCH_GRAB_X_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_GRAB_X_OFFSET", HATCH_GRAB_X_OFFSET);
        HATCH_GRAB_Y_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_GRAB_Y_OFFSET", HATCH_GRAB_Y_OFFSET);
        HATCH_PLACE_X_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_PLACE_X_OFFSET", HATCH_PLACE_X_OFFSET);
        HATCH_PLACE_Y_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/HATCH_PLACE_Y_OFFSET", HATCH_PLACE_Y_OFFSET);
        A = SmartDashboard.getNumber("PercentLineupLocalization/A", A);
        B = SmartDashboard.getNumber("PercentLineupLocalization/B", B);
        C = SmartDashboard.getNumber("PercentLineupLocalization/C", C);
        POINT_DEADZONE = SmartDashboard.getNumber("PercentLineupLocalization/POINT_DEADZONE", POINT_DEADZONE);

//        this.getPIDController().setP(P);
//        this.getPIDController().setI(I);
//        this.getPIDController().setD(D);

        goal = null;

        if (OI.clearBallRocketTargetFlag()) {
            logger.debug("Rocket ball mode!");
            Hardware.limelight.setPipeline(1);
            Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.ROCKET_BALL_TARGET_HEIGHT);
        } else {
            logger.debug("Hatch mode!");
            Hardware.limelight.setPipeline(0);
            Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.HATCH_TARGET_HEIGHT);
        }
        Hardware.limelight.prepForVision();

        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    @Override
    public PointControlConfig getPointControlConfig() {
        return new PointControlConfig(OUTPUT_SCALAR, MIN, MAX, DEADZONE, P, I, D, THROTTLE_CONSTANT);
    }

    @Override
    public double returnAngleError() {
        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
            Transform3D goal = computeGoal();
            if (similarVectorTracker.isSimilarTransform(goal.getPosition())) {
                this.goal = goal;
            } else {
                logger.debug("Ignoring pose estimate- varies by more than the tolerance!");
            }
        }

        if (goal != null) {
            return getAngleError();
        } else {
            return 0;
        }
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
        double distanceToVisionTarget = driveOdometry.getOdomToBaseLink().toTransform2D().getPositionVector().distance(goal.toTransform2D().getPositionVector());
        SmartDashboard.putNumber("DistanceToTarget", distanceToVisionTarget);
        Transform3D adjustedGoal = goal.add(new Transform3D(-(distanceToVisionTarget * B * (1 / (1 + Math.exp(-A * (distanceToVisionTarget - C))))), 0, 0));
        if (distanceToVisionTarget < POINT_DEADZONE) {
            adjustedGoal = goal;
        }
        Vector3D goalPosition = adjustedGoal.getPosition();
        double targetAngle = Math.atan2(goalPosition.getY() - odomPosition.getY(), goalPosition.getX() - odomPosition.getX());
        return TrigUtils.signedAngleError(targetAngle, Hardware.navx.getYawRadians());
    }
}
