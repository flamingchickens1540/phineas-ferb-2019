package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.RobotMap;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryAccumulatorRunnable;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.vision.SimilarVector3DTracker;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class TurnUntilNewTarget extends Command {

    private static final Logger logger = Logger.getLogger(TurnUntilNewTarget.class);
    private static double ANGULAR_VEL = 2.5;
    private final TankDriveTwist2DInput twist2DInput;
    private final Executable pipeline;

    private Transform3D goal;
    private final TankDriveOdometryAccumulatorRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;
    private final boolean left;

    private boolean reset = true;

    private boolean untilAnyTarget;

    private final SimilarVector3DTracker similarVectorTracker = new SimilarVector3DTracker(0.1);

    public TurnUntilNewTarget(TankDriveOdometryAccumulatorRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization, boolean left) {
        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;
        this.left = left;

        requires(Robot.drivetrain);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.putNumber("TurnUntilNewTarget/ANGULAR_VEL", ANGULAR_VEL);
    }

    @Override
    public void initialize() {
        ANGULAR_VEL = SmartDashboard.getNumber("TurnUntilNewTarget/ANGULAR_VEL", ANGULAR_VEL);

        logger.debug("Hatch mode!");
        Hardware.limelight.setPipeline(0);
        Robot.deepSpaceVisionTargetLocalization.setPlaneHeight(RobotMap.HATCH_TARGET_HEIGHT);
        untilAnyTarget = !deepSpaceVisionTargetLocalization.attemptUpdatePose();
        reset = true;
    }

    @Override
    protected void execute() {
        twist2DInput.setTwist(new Twist2D(0, 0, (left ? 1 : -1) * ANGULAR_VEL));
        pipeline.execute();
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget());
    }


    @Override
    protected boolean isFinished() {
        if (OI.getPointDriveMagnitude() > 0.4 || Math.abs(OI.getPointDriveThrottle()) > Tuning.driveDeadzone) {
            return true;
        }
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
