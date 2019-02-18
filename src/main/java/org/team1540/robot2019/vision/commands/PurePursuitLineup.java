package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.LimelightLocalization;
import org.team1540.robot2019.utils.TankDriveOdometryRunnable;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.utils.TrigUtils;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PurePursuitLineup extends Command {

    private final TankDriveOdometryRunnable driveOdometry;
    private final LimelightLocalization limeLoc;

    private double limelightDriveCommand = 0.0;
    private double limelightSteerCommand = 0.0;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Transform3D goal;


    public PurePursuitLineup(LimelightLocalization limeLoc, TankDriveOdometryRunnable driveOdometry) {
        this.limeLoc = limeLoc;
        this.driveOdometry = driveOdometry;
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
//        .then(new FeedForwardProcessor(0.27667, 0.054083,0.08694))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
    }

    @Override
    protected void initialize() {
        Robot.drivetrain.configTalonsForVelocity();
        if (!limeLoc.attemptUpdatePose()) {
        } else {
            goal = computeGoal();
        }
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(limeLoc.getBaseLinkToVisionTarget())
            .add(new Transform3D(-0.55, -0.025, 0));
    }

    @Override
    protected void execute() {
        if (goal == null) {
            return;
        }
        updateLimelight();
        Twist2D cmdVel = new Twist2D(limelightDriveCommand, 0, -limelightSteerCommand);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    public void updateLimelight() {
        if (limeLoc.attemptUpdatePose()) {
            goal = computeGoal();
        }
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = -6;                    // how hard to turn toward the target
        final double DRIVE_K = 4;                    // how hard to drive fwd toward the target
        final double MAX_DRIVE = 0.8;                   // Simple speed limit so we don't drive too fast
        final double MIN_DRIVE = 0.2;
        final double MAX_STEER = 2.0;

        double angleError = getAngleError();
        double distanceError = getDistanceError();

        System.out.printf("Angle error: %f Distance error: %f\n", angleError, distanceError);

        // Start with proportional steering
        double steer_cmd = angleError * STEER_K;
        if (steer_cmd > MAX_STEER) {
            steer_cmd = MAX_STEER;
        } else if (steer_cmd < -MAX_STEER) {
            steer_cmd = -MAX_STEER;
        }
        limelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = distanceError * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        } else if (drive_cmd < MIN_DRIVE) {
            drive_cmd = MIN_DRIVE;
        }

        limelightDriveCommand = drive_cmd;
    }

    @Override
    protected boolean isFinished() {
        if (goal == null) {
            return true;
        }
        if (getDistanceError() < 0.1) {
            Robot.drivetrain.stop();
            return true;
        }
        return false;
    }

    private double getDistanceError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        return goal.toTransform2D().getPositionVector().distance(new Vector2D(odomPosition.getX(), odomPosition.getY()));
    }

    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        return TrigUtils
            .signedAngleDifference(Math.atan2((-goal.toTransform2D().getY()) - (-odomPosition.getY()), goal.toTransform2D().getX() - odomPosition.getX()), Math.toRadians(Robot.navx.getYaw()));
//        return TrigUtils.signedAngleDifference(Math.atan2((-odomPosition.getY())-(-goal.toTransform2D().getY()), odomPosition.getX()-goal.toTransform2D().getX()), Math.toRadians(Robot.navx.getYaw()));
    }
}
