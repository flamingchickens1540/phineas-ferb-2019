package org.team1540.robot2019.vision.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.utils.TrigUtils;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PointLineupSimple extends Command {

    private static final double GOAL_TOLERANCE_ANGULAR = 0.3;
    private static final double MIN_VEL_THETA = 0.1;
    private static final double MAX_VEL_THETA = 2.0;
    private static final double ANGULAR_KP = 10;
    private static final double ANGLE_OFFSET = Math.toRadians(7);

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double goal = null;
    private Transform3D prevGoal = null;

    private boolean endFlag = false;


    public PointLineupSimple() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x == 0) {
            prevGoal = Robot.lastOdomToVisionTarget;
            if (prevGoal == null) {
                Robot.drivetrain.stop();
                System.out.println("Point lineup simple: Unable to find target and no alternative specified");
                return;
            }
            System.out.println("Point lineup simple: Unable to find target. Using alternative goal");
            Vector3D odomPosition = Robot.wheelOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
            goal = prevGoal.toTransform2D().getTheta();
//            goal = -Math.atan2((-prevGoal.toTransform2D().getY()) - (-odomPosition.getY()), prevGoal.toTransform2D().getX() - odomPosition.getX());
            System.out.println(goal);
        } else {
            goal = x - Robot.navx.getYawRadians();
            System.out.println("Point lineup simple starting");
        }
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
        Robot.drivetrain.configTalonsForVelocity();
    }

    @Override
    protected void execute() {
        if (goal == null) {
            return;
        }
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x != 0) {
            goal = -(x - ANGLE_OFFSET) + Robot.navx.getYawRadians();
        }

        double angleError = getAngleError(goal);

        double cmdVelTheta = angleError * ANGULAR_KP;
        if (cmdVelTheta > MAX_VEL_THETA) {
            cmdVelTheta = MAX_VEL_THETA;
        } else if (cmdVelTheta < -MAX_VEL_THETA) {
            cmdVelTheta = -MAX_VEL_THETA;
        } else if (cmdVelTheta > -MIN_VEL_THETA && cmdVelTheta < MIN_VEL_THETA) {
            cmdVelTheta = Math.copySign(MIN_VEL_THETA, cmdVelTheta);
        }
        System.out.printf("Angle error: %f Steer command: %f\n", angleError, cmdVelTheta);

        Twist2D cmdVel = new Twist2D(0, 0, cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    @Override
    protected boolean isFinished() {
        System.out.println(Robot.drivetrain.getTwist().getOmega());
        return goal == null ||
            (Math.abs(getAngleError(goal)) < Math.toRadians(GOAL_TOLERANCE_ANGULAR)
                && Math.abs(Robot.drivetrain.getTwist().getOmega()) < 0.2);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Robot.navx.getYawRadians());
    }
}
