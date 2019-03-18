package org.team1540.robot2019.commands.drivetrain;

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

public class PointDrive extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PointDrive.class);

    // Max/Min angular velocity
    private static final double MIN = 0;
    private static final double MAX = 10;
    private static final double DEADZONE = 0.05;

    private static final double OUTPUT_SCALAR = 20;

    // Constants for angular PID controller
    private static final double P = 0.2;
    private static final double I = 0;
    private static final double D = 0.5;

    private static final double POINT_JOYSTICK_DEADZONE = 0.5;
    private static final double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    private static Double initAngleOffset = null;
    private static Double goalAngle = null;

    private final TankDriveTwist2DInput twist2DInput;
    private final Executable pipeline;

    public PointDrive() {
        super(
            P,
            I,
            D
        );
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));
        if (PointDrive.initAngleOffset == null) { // Only set initial angle offset if there is no offset already set
            setInitAngleOffset(Hardware.navx.getYawRadians());
        }
        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
    }

    @Override
    protected void initialize() {
        setGoalToCurrentAngle();
    }

    private void setGoalToCurrentAngle() {
        goalAngle = Hardware.navx.getYawRadians() - initAngleOffset;
    }

    public static void setInitAngleOffset(Double initAngleOffset) {
        goalAngle = null;
        PointDrive.initAngleOffset = initAngleOffset;
    }

    @Override
    protected double returnPIDInput() {
        if (OI.getPointDriveMagnitude() > POINT_JOYSTICK_DEADZONE) {
            goalAngle = OI.getPointDriveAngle();
        }
        if (goalAngle == null) {
            return 0;
        }
        return getAngleError(goalAngle + initAngleOffset);
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= OUTPUT_SCALAR;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX, MIN);
        if (goalAngle == null || Math.abs(output) < DEADZONE) {
            cmdVelTheta = 0;
        }
        Twist2D cmdVel = new Twist2D(-OI.getTankdriveLeftAxis() * THROTTLE_CONSTANT, 0, -cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }

    private double getAngleError(double x) {
        return TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
