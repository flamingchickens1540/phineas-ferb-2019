package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
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

public class PointLineup extends Command {

    private final TankDriveOdometryRunnable driveOdometry;
    private final LimelightLocalization limeLoc;

    private double limelightDriveCommand = 0.0;
    private double limelightSteerCommand = 0.0;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Transform3D goal;

    private static final Transform3D offset = new Transform3D(0, 0, 0);
//    private static final Transform3D offset = new Transform3D(0, -0.025, 0);


    public PointLineup(LimelightLocalization limeLoc, TankDriveOdometryRunnable driveOdometry, Transform3D lastBaseLinkToVisionTarget) {
        this.limeLoc = limeLoc;
        this.driveOdometry = driveOdometry;
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
//        .then(new FeedForwardProcessor(0.27667, 0.054083,0.08694))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
        if (lastBaseLinkToVisionTarget != null) {
            goal = lastBaseLinkToVisionTarget
                .add(offset);
        }
    }

//    public PointLineup(Transform3D initalGoalEstimate, LimelightLocalization limeLoc, TankDriveOdometryRunnable driveOdometry, Transform3D lastOdomToVisionTarget) {
//        this.limeLoc = limeLoc;
//        this.driveOdometry = driveOdometry;
//        this.lastOdomToVisionTarget = lastOdomToVisionTarget;
//        this.goal = initalGoalEstimate;
//        requires(Robot.drivetrain);
//        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
//        pipeline = twist2DInput
//            .then(new FeedForwardProcessor(0, 0, 0))
////        .then(new FeedForwardProcessor(0.27667, 0.054083,0.08694))
//            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
//            .then(Robot.drivetrain.getPipelineOutput());
//    }

    @Override
    protected void initialize() {
        Robot.drivetrain.configTalonsForVelocity();
        if (goal == null) {
            if (!limeLoc.attemptUpdatePose()) {
                System.out.println("Vision target not found!");
            } else {
                goal = computeGoal();
            }
        }
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(limeLoc.getBaseLinkToVisionTarget())
            .add(offset);
    }

    @Override
    protected void execute() {
        if (goal == null) {
            return;
        }
        updateLimelight();
        Twist2D cmdVel = new Twist2D(0, 0, -limelightSteerCommand);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    public void updateLimelight() {
        if (limeLoc.attemptUpdatePose()) {
            goal = computeGoal();
        }
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = -10;                    // how hard to turn toward the target
        final double MAX_STEER = 2.0;
        final double MIN_STEER = 0.5;

        double angleError = getAngleError();

        // Start with proportional steering
        double steer_cmd = angleError * STEER_K;
        if (steer_cmd > MAX_STEER) {
            steer_cmd = MAX_STEER;
        } else if (steer_cmd < -MAX_STEER) {
            steer_cmd = -MAX_STEER;
        } else if (steer_cmd > -MIN_STEER && steer_cmd < MIN_STEER) {
            steer_cmd = Math.copySign(MIN_STEER, steer_cmd);
        }
        System.out.printf("Angle error: %f Steer command: %f\n", angleError, steer_cmd);
        limelightSteerCommand = steer_cmd;
    }

    @Override
    protected boolean isFinished() {
        if (goal == null) {
            System.out.println("Vision target not found and no goal given!");
            Robot.drivetrain.stop();
            return true;
        }
        if (Math.abs(getAngleError()) < Math.toRadians(0.3)) {
            System.out.println("Goal reached!");
            Robot.drivetrain.stop();
            return true;
        }
        return false;
    }

    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        return TrigUtils
            .signedAngleDifference(Math.atan2((-goal.toTransform2D().getY()) - (-odomPosition.getY()), goal.toTransform2D().getX() - odomPosition.getX()), Math.toRadians(Robot.navx.getYaw()));
//        return TrigUtils.signedAngleDifference(Math.atan2((-odomPosition.getY())-(-goal.toTransform2D().getY()), odomPosition.getX()-goal.toTransform2D().getX()), Math.toRadians(Robot.navx.getYaw()));
    }
}
