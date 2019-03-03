package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class SimplePointToVisionTarget extends PIDCommand {

    public static final Logger logger = Logger.getLogger(SimplePointToVisionTarget.class);

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 0.2;
    private static final double MAX_VEL_THETA = 1.8;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -6;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -10;

    private static final double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    // Goal tolerances for angle
    private static final double GOAL_TOLERANCE_ANGULAR_POSITION = Math.toRadians(0.6);
    private static final double GOAL_TOLERANCE_ANGULAR_VELOCITY = 0.3;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double goal = null;

    public SimplePointToVisionTarget() { // TODO: Separate vision processing and point to angle
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
        double x = Robot.limelight.getTargetAngles().getX();
        if (x == 0) {
            Transform3D prevGoal = Robot.lastOdomToVisionTargetTracker.getOdomToVisionTarget();
            if (prevGoal == null) {
                Robot.drivetrain.stop();
                logger.warn("Unable to find target and no alternative specified! Ending...");
                return;
            }
            goal = prevGoal.toTransform2D().getTheta();
            logger.debug("Unable to find target. Using alternative goal angle: " + goal);
        } else {
            goal = -(x - ANGLE_OFFSET) + Hardware.navx.getYawRadians(); // TODO: This should not be duplicated twice
            logger.debug("Point lineup simple starting. Initial goal angle: " + goal);
        }
    }

    @Override
    protected void execute() {
        if (goal == null) {
            return;
        }
        double x = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight-a").getEntry("tx").getDouble(0));
        if (x != 0) {
            goal = -(x - ANGLE_OFFSET) + Hardware.navx.getYawRadians();
        }
    }

    @Override
    protected boolean isFinished() {
        if (goal == null) {
            return true;
        }
        double anglePosError = Math.abs(getAngleError(goal));
        double angleVelError = Math.abs(Robot.drivetrain.getTwist().getOmega());
        boolean isFinished = anglePosError < GOAL_TOLERANCE_ANGULAR_POSITION && angleVelError < GOAL_TOLERANCE_ANGULAR_VELOCITY;
        if (isFinished) {
            logger.debug(String.format("Simple point goal reached! Angle error remaining: %f Angular velocity error remaining: %f", anglePosError, angleVelError));
        }
        return isFinished;
    }

    @Override
    protected void end() {
        logger.debug("SimplePointToTarget Ended!");
        Robot.drivetrain.stop();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }

    @Override
    protected double returnPIDInput() {
        if (goal == null) {
            return 0;
        }
        return getAngleError(goal);
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
