package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.vision.SimilarVector3DTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class TurnUntilNewTarget extends Command {

    private static final Logger logger = Logger.getLogger(TurnUntilNewTarget.class);
    private static final double MAX_ANGULAR_VEL = 5;
    private static double FAST_ANGULAR_VEL = 7;
    private static double SLOW_ANGULAR_VEL = 2.5;
    private static double FAST_TIMEOUT_K = 0.075;
    private final TankDriveTwist2DInput twist2DInput;
    private final Executable pipeline;

    private Transform3D goal;
    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
    private Boolean left = null;

    private boolean reset = true;

    private boolean untilAnyTarget;

    private final SimilarVector3DTracker similarVectorTracker = new SimilarVector3DTracker(0.1);
    private double fastTimeout;

    public TurnUntilNewTarget(TankDriveOdometryAccumulatorRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization, boolean left) {
        this(driveOdometry, deepSpaceVisionTargetLocalization);
        this.left = left;
    }

    public TurnUntilNewTarget(TankDriveOdometryAccumulatorRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization) {
        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;

        requires(Robot.drivetrain);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.putNumber("TurnUntilNewTarget/FAST_ANGULAR_VEL", FAST_ANGULAR_VEL);
        SmartDashboard.putNumber("TurnUntilNewTarget/SLOW_ANGULAR_VEL", SLOW_ANGULAR_VEL);
        SmartDashboard.putNumber("TurnUntilNewTarget/FAST_TIMEOUT_K", FAST_TIMEOUT_K);
    }

    @Override
    public void initialize() {
        FAST_ANGULAR_VEL = SmartDashboard.getNumber("TurnUntilNewTarget/FAST_ANGULAR_VEL", FAST_ANGULAR_VEL);
        SLOW_ANGULAR_VEL = SmartDashboard.getNumber("TurnUntilNewTarget/SLOW_ANGULAR_VEL", SLOW_ANGULAR_VEL);
        FAST_TIMEOUT_K = SmartDashboard.getNumber("TurnUntilNewTarget/FAST_TIMEOUT_K", FAST_TIMEOUT_K);

        double distanceToVisionTarget = Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget();
        fastTimeout = FAST_TIMEOUT_K / distanceToVisionTarget;

        SmartDashboard.putNumber("TurnUntilNewTarget/distanceToVisionTarget", distanceToVisionTarget);
        SmartDashboard.putNumber("TurnUntilNewTarget/fastTimeout", fastTimeout);


        logger.debug("Hatch mode!");
        Hardware.limelight.setPipeline(0);
        Hardware.limelight.setLeds(true);
        untilAnyTarget = !deepSpaceVisionTargetLocalization.attemptUpdatePose();
        reset = true;
    }

    @Override
    protected void execute() {
        double omega = 0;
        if (left != null) {
            omega = (left ? 1 : -1) * ControlUtils.linearDeadzoneRamp(fastTimeout - this.timeSinceInitialized(), false, FAST_ANGULAR_VEL, SLOW_ANGULAR_VEL, 0.02, -0.02);
        } else {
            omega = 0;//ControlUtils.velocityPosNegConstrain(OI.getPointUntilNextTargetAxis() * MAX_ANGULAR_VEL, MAX_ANGULAR_VEL, SLOW_ANGULAR_VEL);
        }
        SmartDashboard.putNumber("TurnUntilNewTarget/omega", omega);
        twist2DInput.setTwist(new Twist2D(0, 0, omega));
        pipeline.execute();
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget());
    }


    @Override
    protected boolean isFinished() {
//        if (OI.getPointDriveMagnitude() > 0.4 || Math.abs(OI.getPointDriveThrottle()) > Tuning.driveDeadzone) {
//            return true;
//        }
        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
            if (untilAnyTarget) {
                return true;
            }
            Vector3D position = computeGoal().getPosition();
            if (reset) {
                reset = false;
                similarVectorTracker.setVector3D(position);
            }
            return !similarVectorTracker.isSimilarTransform(position);
        }
        return false;
    }

    @Override
    protected void end() {
        Robot.drivetrain.getDriveCommand().pointNextReset();
    }

}
