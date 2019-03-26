package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.vision.SimilarVector3DTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;

public class PercentManualLineupLocalization extends PointManualDriveCommand {

    public static final Logger logger = Logger.getLogger(PercentManualLineupLocalization.class);
    private static final Transform3D VISION_TARGET_OFFSET = Transform3D.IDENTITY;

    private static double OUTPUT_SCALAR = 20;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular VPID controller
    private static double P = 0.32;
    private static double I = 0;
    private static double D = 0.6;

    public static double ANGLE_OFFSET = 0; // Degrees offset from center of target
//    public static double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    private Transform3D goal;
    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;

    private final SimilarVector3DTracker similarVectorTracker = new SimilarVector3DTracker(0.3);

    public PercentManualLineupLocalization(TankDriveOdometryAccumulatorRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization) {
        super(P, I, D, OUTPUT_SCALAR, MAX, MIN, DEADZONE, THROTTLE_CONSTANT);
        requires(Robot.drivetrain);

        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;

        SmartDashboard.putNumber("PercentLineupLocalization/OUTPUT_SCALAR", OUTPUT_SCALAR);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KP", P);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KI", I);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGULAR_KD", D);
        SmartDashboard.putNumber("PercentLineupLocalization/MAX_VEL_THETA", MAX);
        SmartDashboard.putNumber("PercentLineupLocalization/MIN_VEL_THETA", MIN);
        SmartDashboard.putNumber("PercentLineupLocalization/DEADZONE_VEL_THETA", DEADZONE);
        SmartDashboard.putNumber("PercentLineupLocalization/ANGLE_OFFSET", ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        P = SmartDashboard.getNumber("PercentLineupLocalization/OUTPUT_SCALAR", OUTPUT_SCALAR);
        P = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KP", P);
        I = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KI", I);
        D = SmartDashboard.getNumber("PercentLineupLocalization/ANGULAR_KD", D);
        MAX = SmartDashboard.getNumber("PercentLineupLocalization/MAX_VEL_THETA", MAX);
        MIN = SmartDashboard.getNumber("PercentLineupLocalization/MIN_VEL_THETA", MIN);
        DEADZONE = SmartDashboard.getNumber("PercentLineupLocalization/DEADZONE_VEL_THETA", DEADZONE);
        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineupLocalization/ANGLE_OFFSET", ANGLE_OFFSET);

        this.getPIDController().setP(P);
        this.getPIDController().setI(I);
        this.getPIDController().setD(D);

        similarVectorTracker.reset();
        goal = null;

        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    @Override
    protected double returnAngleError() {
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
        return driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget())
            .add(VISION_TARGET_OFFSET);
    }


    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        double distanceToVisionTarget = driveOdometry.getOdomToBaseLink().getPosition().distance(goal.getPosition());
        Transform3D adjustedGoal = goal.add(new Transform3D(-(distanceToVisionTarget*0.5), 0, 0));
        Vector3D goalPosition = adjustedGoal.getPosition();
        double targetAngle = Math.atan2(goalPosition.getY() - odomPosition.getY(), goalPosition.getX() - odomPosition.getX());
        return TrigUtils.signedAngleError(targetAngle, Hardware.navx.getYawRadians());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        logger.debug("Ended");
    }
}
