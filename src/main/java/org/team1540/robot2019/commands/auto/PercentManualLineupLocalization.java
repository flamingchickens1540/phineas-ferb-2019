package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Hardware;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.threed.Transform3D;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.datastructures.utils.TrigUtils;
import org.team1540.robot2019.odometry.tankdrive.TankDriveOdometryRunnable;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.robot2019.vision.deepspace.DeepSpaceVisionTargetLocalization;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class PercentManualLineupLocalization extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PercentManualLineup.class);

    private static double OUTPUT_SCALAR = 20;

    private static double DEADZONE_VEL_THETA = 0.05;

    // Max/Min angular velocity
    private static double MIN_VEL_THETA = 0;
    private static double MAX_VEL_THETA = 10;

    // Constants for angular VPID controller
    private static double ANGULAR_KP = 0.32;
    private static double ANGULAR_KI = 0;
    private static double ANGULAR_KD = 0.6;
//
//    // Max/Min angular velocity
//    public static double MAX_VEL_THETA = 1;
//    public static double MIN_VEL_THETA = 0.06;
//
//    public static double DEADZONE_VEL_THETA = 0.005;
//
//    // Constants for angular VPID controller
//    public static double ANGULAR_KP = -0.55;
//    public static double ANGULAR_KI = 0;
//    public static double ANGULAR_KD = -2;

    public static double ANGLE_OFFSET = 0; // Degrees offset from center of target
//    public static double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Transform3D goal = null;
    private final TankDriveOdometryRunnable driveOdometry;
    private final DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization;

    public PercentManualLineupLocalization(TankDriveOdometryRunnable driveOdometry, DeepSpaceVisionTargetLocalization deepSpaceVisionTargetLocalization) {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        this.driveOdometry = driveOdometry;
        this.deepSpaceVisionTargetLocalization = deepSpaceVisionTargetLocalization;
        System.out.printf("Config updated: P: %f I: %f D: %f Max: %f Min: %f", ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, MAX_VEL_THETA, MIN_VEL_THETA);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.setDefaultNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KP", ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KI", ANGULAR_KI);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KD", ANGULAR_KD);
        SmartDashboard.setDefaultNumber("PercentLineup/MAX_VEL_THETA", MAX_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/MIN_VEL_THETA", MIN_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR); // TODO: Remove temporary tuning (yaml ftw)
        ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/ANGULAR_KP", ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        ANGULAR_KI = SmartDashboard.getNumber("PercentLineup/ANGULAR_KI", ANGULAR_KI);
        ANGULAR_KD = SmartDashboard.getNumber("PercentLineup/ANGULAR_KD", ANGULAR_KD);
        MAX_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MAX_VEL_THETA", MAX_VEL_THETA);
        MIN_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MIN_VEL_THETA", MIN_VEL_THETA);
        DEADZONE_VEL_THETA = SmartDashboard.getNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE_VEL_THETA);
        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);

        this.getPIDController().setP(ANGULAR_KP);
        this.getPIDController().setI(ANGULAR_KI);
        this.getPIDController().setD(ANGULAR_KD);

        logger.debug("Starting...");
    }

    private Transform3D computeGoal() {
        return driveOdometry.getOdomToBaseLink()
            .add(deepSpaceVisionTargetLocalization.getLastBaseLinkToVisionTarget());
//            .add(VISION_TARGET_OFFSET);
    }


    private double getAngleError() {
        Vector3D odomPosition = driveOdometry.getOdomToBaseLink().getPosition(); // TODO: This should use javaTF
        double targetAngle = Math.atan2(goal.toTransform2D().getY() - odomPosition.getY(), goal.toTransform2D().getX() - odomPosition.getX());
        double currentAngle = Hardware.navx.getYawRadians();
        return TrigUtils.signedAngleError(targetAngle, currentAngle);
    }

    @Override
    protected double returnPIDInput() {
        if (deepSpaceVisionTargetLocalization.attemptUpdatePose()) {
            goal = computeGoal();
        }

        if (goal != null && Robot.limelight.isTargetFound()) {
            return getAngleError();
        } else {
            return 0;
        }
    }

    @Override
    protected void usePIDOutput(double output) {
        output *= OUTPUT_SCALAR;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        if (Math.abs(output) < DEADZONE_VEL_THETA) {
            cmdVelTheta = 0;
        }

        // no ff
//        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -0.7, 0, cmdVelTheta);
//        Robot.drivetrain.setPercentTwist(cmdVel);

        // ff
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -3, 0, -cmdVelTheta);
        twist2DInput.setTwist(cmdVel);
        pipeline.execute();
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        logger.debug("Ended");
    }
}
