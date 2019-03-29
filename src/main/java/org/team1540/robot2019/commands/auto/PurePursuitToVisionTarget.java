//package org.team1540.robot2019.commands.auto;
//
//import edu.wpi.first.wpilibj.command.Command;
//import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
//import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
//import org.apache.log4j.Logger;
//import org.team1540.robot2019.Hardware;
//import org.team1540.robot2019.Robot;
//import org.team1540.robot2019.Tuning;
//import org.team1540.robot2019.datastructures.threed.Transform3D;
//import org.team1540.robot2019.datastructures.twod.Twist2D;
//import org.team1540.robot2019.datastructures.utils.TrigUtils;
//import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
//import org.team1540.robot2019.utils.ControlUtils;
//import org.team1540.robot2019.utils.TankDriveTwist2DInput;
//import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
//import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
//import org.team1540.rooster.drive.pipeline.UnitScaler;
//import org.team1540.rooster.functional.Executable;
//
//// TODO: Use proper logging class
//// TODO: Make this a generic pure pursuit command
//public class PurePursuitToVisionTarget extends Command {
//
//    private static final Logger logger = Logger.getLogger(PurePursuitToVisionTarget.class);
//
//    private static final double ANGULAR_KP = 5;
//    private static final double LINEAR_KP = 4;
//    private static final double MAX_VEL_X = 0.8;
//    private static final double MIN_VEL_X = 0.2;
//    private static final double MAX_VEL_THETA = 2.0;
//    private static final double GOAL_DISTANCE_TOLERANCE = 0.05;
//    private static final Transform3D VISION_TARGET_OFFSET = new Transform3D(-0.65, -0.025, 0);
//
//    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
//    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
//    private Runnable onFail = null;
//
//    private Executable pipeline;
//    private TankDriveTwist2DInput twist2DInput;
//    private Transform3D goal;
//
//    public PurePursuitToVisionTarget(DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization, TankDriveOdometryAccumulatorRunnable driveOdometry) {
//        requires(Robot.drivetrain);
//        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;
//        this.driveOdometry = driveOdometry;
//        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
//        pipeline = twist2DInput
//            .then(new FeedForwardProcessor(0, 0, 0))
//            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
//            .then(Robot.drivetrain.getPipelineOutput());
//    }
//
//    public PurePursuitToVisionTarget(DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization, TankDriveOdometryAccumulatorRunnable driveOdometry, Runnable onFail) {
//        this(deepSpaceVisionTargetLocalization, driveOdometry);
//        this.onFail = onFail;
//    }
//
//    @Override
//    protected void initialize() {
//        logger.debug("Pure pursuit starting...");
//        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
//            logger.debug("Vision target pose acquired!");
//            goal = computeGoal();
//        } else {
//            logger.warn("Unable to get vision target pose!");
//            if (onFail != null) {
//                logger.debug("Calling onFail method!");
//                onFail.run();
//            } else {
//                logger.debug("No onFail method specified!");
//            }
//        }
//    }
//
//    private Transform3D computeGoal() {
//        return driveOdometry.getOdomToBaseLink()
//            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget())
//            .add(VISION_TARGET_OFFSET);
//    }
//
//    @Override
//    protected void execute() {
//        if (goal == null) {
//            return;
//        }
//        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
//            goal = computeGoal();
//        }
//
//        double angleError = getAngleError();
//        double distanceError = getDistanceError();
//
//        double cmdVelTheta = angleError * ANGULAR_KP; // TODO: Replace this with ControlUtils.velocityPosNegConstrain
//        cmdVelTheta = ControlUtils.velocityPosNegConstrain(cmdVelTheta, MAX_VEL_THETA, 0);
//
//        double cmdVelX = distanceError * (1 - Math.abs(angleError) / Math.PI * 2) * LINEAR_KP;
//        cmdVelX = ControlUtils.velocityPosNegConstrain(cmdVelX, MAX_VEL_X, MIN_VEL_X);
//
//        Twist2D cmdVel = new Twist2D(cmdVelX, 0, cmdVelTheta);
//        twist2DInput.setTwist(cmdVel);
//        pipeline.execute();
//    }
//
//    @Override
//    protected boolean isFinished() {
//        if (goal == null) {
//            return true;
//        }
//        double distanceError = getDistanceError();
//        boolean isFinished = distanceError < GOAL_DISTANCE_TOLERANCE;
//        if (isFinished) {
//            logger.debug("Pure pursuit goal reached! Distance error remaining: " + distanceError);
//        }
//        return isFinished;
//    }
//
//    @Override
//    protected void end() {
//        Robot.drivetrain.stop();
//    }
//
//    private double getDistanceError() {
//        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
//        return goal.toTransform2D().getPositionVector().distance(new Vector2D(odomPosition.getX(), odomPosition.getY()));
//    }
//
//    private double getAngleError() {
//        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
//        double targetAngle = Math.atan2(goal.toTransform2D().getY() - odomPosition.getY(), goal.toTransform2D().getX() - odomPosition.getX());
//        double currentAngle = Hardware.navx.getYawRadians();
//        return TrigUtils.signedAngleError(targetAngle, currentAngle);
//    }
//}
