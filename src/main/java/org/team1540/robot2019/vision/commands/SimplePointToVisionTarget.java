package org.team1540.robot2019.vision.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.utils.TrigUtils;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class SimplePointToVisionTarget extends PIDCommand {

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 0.1;
    private static final double MAX_VEL_THETA = 1.8;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -6;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -10;

    private static final double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    // Goal tolerances for angle
    private static final double GOAL_TOLERANCE_ANGULAR_POSITION = 0.6;
    private static final double GOAL_TOLERANCE_ANGULAR_VELOCITY = 0.3;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double goal = null;
    private Transform3D prevGoal = null;

    public SimplePointToVisionTarget() {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
    }

    @Override
    protected void initialize() {
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x == 0) {
            prevGoal = Robot.lastOdomToVisionTarget; // TODO: This should not rely on others to update vision target pose
            if (prevGoal == null) {
                Robot.drivetrain.stop();
                System.out.println("Point lineup simple: Unable to find target and no alternative specified");
                return;
            }
            System.out.println("Point lineup simple: Unable to find target. Using alternative goal");
            Vector3D odomPosition = Robot.wheelOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
            goal = prevGoal.toTransform2D().getTheta();
        } else {
            goal = x - Robot.navx.getYawRadians();
            System.out.println("Point lineup simple starting");
        }
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
    }

    @Override
    protected boolean isFinished() {
        return goal == null ||
            (Math.abs(getAngleError(goal)) < Math.toRadians(GOAL_TOLERANCE_ANGULAR_POSITION)
                && Math.abs(Robot.drivetrain.getTwist().getOmega()) < GOAL_TOLERANCE_ANGULAR_VELOCITY);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Robot.navx.getYawRadians());
    }

    @Override
    protected double returnPIDInput() {
        double angleError = getAngleError(goal);

        return angleError;
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= MAX_VEL_THETA;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);

        Twist2D cmdVel = new Twist2D(0, 0, cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }
}
