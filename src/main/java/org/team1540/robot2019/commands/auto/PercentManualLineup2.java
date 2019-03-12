package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryRunnable;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PercentManualLineup2 extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PercentManualLineup2.class);

    // Max/Min angular velocity
    public static double MIN_VEL_THETA = 0.05;
    public static double MAX_VEL_THETA = 1;

    public static double DEADZONE_VEL_THETA = 0.01;

    // Constants for angular VPID controller
    public static double ANGULAR_KP = -0.55;
    public static double ANGULAR_KI = 0;
    public static double ANGULAR_KD = -2;

    public static double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Transform3D goal = null;
    private final TankDriveOdometryRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;

    public PercentManualLineup2(TankDriveOdometryRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization) {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;
        System.out.printf("Config updated: P: %f I: %f D: %f Max: %f Min: %f", ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, MAX_VEL_THETA, MIN_VEL_THETA);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
    }

    @Override
    protected void initialize() {
        this.getPIDController().setP(ANGULAR_KP); // TODO: Is this allowed??
        this.getPIDController().setI(ANGULAR_KI);
        this.getPIDController().setD(ANGULAR_KD);
        logger.debug("Pure pursuit starting...");
        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
            logger.debug("Vision target pose acquired!");
            goal = computeGoal();
        } else {
            logger.warn("Unable to get vision target pose!");
//            if (onFail != null) {
//                logger.debug("Calling onFail method!");
//                onFail.run();
//            } else {
//                logger.debug("No onFail method specified!");
//            }
        }
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget());
//            .add(VISION_TARGET_OFFSET);
    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return goal == null;
    }

    @Override
    protected void end() {
        logger.debug("SimplePointToTarget Ended!");
    }

//    private double getDistanceError() {
//        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
//        return goal.toTransform2D().getPositionVector().distance(new Vector2D(odomPosition.getX(), odomPosition.getY()));
//    }

    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        double targetAngle = Math.atan2(goal.toTransform2D().getY() - odomPosition.getY(), goal.toTransform2D().getX() - odomPosition.getX());
        double currentAngle = Hardware.navx.getYawRadians();
        return TrigUtils.signedAngleError(targetAngle, currentAngle);
    }

    @Override
    protected double returnPIDInput() {
        if (goal == null) {
            return 0;
        }

        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
            goal = computeGoal();
        }

        return getAngleError();
    }

    @Override
    protected void usePIDOutput(double output) {
//        output *= MAX_VEL_THETA;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        if (Math.abs(output) < DEADZONE_VEL_THETA) {
            cmdVelTheta = 0;
        }
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -0.7, 0, cmdVelTheta);
        Robot.drivetrain.setPercentTwist(cmdVel);
//        twist2DInput.setTwist(cmdVel);
//        pipeline.execute();
    }
}
