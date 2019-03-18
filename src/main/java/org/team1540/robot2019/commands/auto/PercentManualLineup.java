package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
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

public class PercentManualLineup extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PercentManualLineup.class);

    private static double OUTPUT_SCALAR = 20;

    private static double DEADZONE = 0.05;

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;

    // Constants for angular VPID controller
    private static double ANGULAR_KP = 0.32;
    private static double ANGULAR_KI = 0;
    private static double ANGULAR_KD = 0.6;

    private static final double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    //        public static double ANGLE_OFFSET = 0; // Degrees offset from center of target
    public static double ANGLE_OFFSET = Math.toRadians(8); // Degrees offset from center of target

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    public PercentManualLineup() {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.putNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KP", ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KI", ANGULAR_KI);
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KD", ANGULAR_KD);
        SmartDashboard.putNumber("PercentLineup/MAX_VEL_THETA", MAX);
        SmartDashboard.putNumber("PercentLineup/MIN_VEL_THETA", MIN);
        SmartDashboard.putNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE);
        SmartDashboard.putNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR); // TODO: Remove temporary tuning (yaml ftw)
        ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/ANGULAR_KP", ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        ANGULAR_KI = SmartDashboard.getNumber("PercentLineup/ANGULAR_KI", ANGULAR_KI);
        ANGULAR_KD = SmartDashboard.getNumber("PercentLineup/ANGULAR_KD", ANGULAR_KD);
        MAX = SmartDashboard.getNumber("PercentLineup/MAX_VEL_THETA", MAX);
        MIN = SmartDashboard.getNumber("PercentLineup/MIN_VEL_THETA", MIN);
        DEADZONE = SmartDashboard.getNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE);
        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);

        this.getPIDController().setP(ANGULAR_KP);
        this.getPIDController().setI(ANGULAR_KI);
        this.getPIDController().setD(ANGULAR_KD);

        logger.debug(String.format("Initialized with constants: P: %f I: %f D: %f Max: %f Min: %f", ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, MAX, MIN));
        logger.debug("Starting...");
    }

    @Override
    protected double returnPIDInput() {
        if (Robot.limelight.isTargetFound()) {
            double angleToVisionTarget = -Robot.limelight.getTargetAngles().getX(); // TODO: Use limelight interface
            return TrigUtils.signedAngleError(0, angleToVisionTarget + ANGLE_OFFSET);
        } else {
            return 0;
        }
    }

    @Override
    protected void usePIDOutput(double output) {
        double cmdVelTheta = ControlUtils.allVelocityConstraints(output*OUTPUT_SCALAR, MAX, MIN, DEADZONE);
        twist2DInput.setTwist(new Twist2D(OI.getPointDriveThrottle() * THROTTLE_CONSTANT, 0, -cmdVelTheta));
        pipeline.execute();
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        logger.debug("Ended!");
    }
}
