package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class SimplePointToAngle extends PIDCommand {

    public static final Logger logger = Logger.getLogger(SimplePointToAngle.class);

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 1.5;
    private static final double MAX_VEL_THETA = 6;

    private static final double OUTPUT_SCALAR = 5;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -1;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -2;

    // Goal tolerances for angle
    private static final double GOAL_TOLERANCE_ANGULAR_POSITION = Math.toRadians(5);
    private static final double GOAL_TOLERANCE_ANGULAR_VELOCITY = Double.POSITIVE_INFINITY;
//    private static final double GOAL_TOLERANCE_ANGULAR_VELOCITY = 0.3;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double absoluteGoal;
    private Double relativeGoal;

    public SimplePointToAngle(double goalAngle) {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());
        relativeGoal = goalAngle;
    }

    @Override
    protected void initialize() {
        absoluteGoal = relativeGoal + Hardware.navx.getYawRadians();
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        double anglePosError = Math.abs(getAngleError(absoluteGoal));
        double angleVelError = Math.abs(Robot.drivetrain.getTwist().getOmega());
        boolean isFinished = anglePosError < GOAL_TOLERANCE_ANGULAR_POSITION && angleVelError < GOAL_TOLERANCE_ANGULAR_VELOCITY;
        if (isFinished) {
            logger.debug(String.format("Goal reached! Angle error remaining: %f Angular velocity error remaining: %f", anglePosError, angleVelError));
        }
        return isFinished;
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }

    @Override
    protected double returnPIDInput() {
        return getAngleError(absoluteGoal);
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= OUTPUT_SCALAR;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -0.7, 0, cmdVelTheta * 0.1);
        Robot.drivetrain.setPercentTwist(cmdVel);
//        twist2DInput.setTwist(cmdVel);
//        pipeline.execute();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }
}
