package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
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

public class SimplePointToVisionTargetAndDrive extends PIDCommand {

    public static final Logger logger = Logger.getLogger(SimplePointToVisionTargetAndDrive.class);

    // Max/Min angular velocity
    private static final double MIN_VEL_THETA = 0.1;
    private static final double MAX_VEL_THETA = 1.8;

    // Constants for angular VPID controller
    private static final double ANGULAR_KP = -6;
    private static final double ANGULAR_KI = 0;
    private static final double ANGULAR_KD = -10;

    private static final double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double goal = null;

    public SimplePointToVisionTargetAndDrive() {
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
        if (!Robot.limelight.isTargetFound() || x == 0) {
            goal = null;
            logger.warn("Unable to find target and no alternative specified! Ending...");
        } else {
            goal = x - Hardware.navx.getYawRadians();
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
        return goal == null;
    }

    @Override
    protected void end() {
        logger.debug("SimplePointToTarget Ended!");
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

        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis()*-1.3, 0, cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }
}
