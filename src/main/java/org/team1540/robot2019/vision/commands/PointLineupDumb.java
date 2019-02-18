package org.team1540.robot2019.vision.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.utils.TrigUtils;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PointLineupDumb extends Command {

    private double limelightSteerCommand = 0.0;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private double goal;

    private boolean endFlag = false;


    public PointLineupDumb() {
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
//        .then(new FeedForwardProcessor(0.27667, 0.054083,0.08694))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x == 0) {
            Robot.drivetrain.stop();
            endFlag = true;
        } else {
            goal = x - Math.toRadians(-Robot.navx.getYaw());
        }
    }

    public PointLineupDumb(double goal) {
        this.goal = goal;
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
    }

    @Override
    protected void execute() {
        updateLimelight();
        Twist2D cmdVel = new Twist2D(0, 0, limelightSteerCommand);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    public void updateLimelight() {
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x != 0) {
//        if (goal == 0) {
            goal = -(x - Math.toRadians(7)) + Math.toRadians(-Robot.navx.getYaw());
        }
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = -10;                    // how hard to turn toward the target
        final double MAX_STEER = 2.0;
        final double MIN_STEER = 0.3;

        double angleError = getAngleError(goal);

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
        if (endFlag) {
            Robot.drivetrain.stop();
            return true;
        }
        if (Math.abs(getAngleError(goal)) < Math.toRadians(0.3)) {
            Robot.drivetrain.stop();
            return true;
        }
        return false;
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleDifference(x, Math.toRadians(-Robot.navx.getYaw()));
    }
}
