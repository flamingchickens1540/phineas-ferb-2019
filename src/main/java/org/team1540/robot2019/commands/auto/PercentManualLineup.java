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

    // Max/Min angular velocity
    private static double MIN = 0;
    private static double MAX = 10;
    private static double DEADZONE = 0.05;

    // Constants for angular PID controller
    private static double P = 0.32;
    private static double I = 0;
    private static double D = 0.6;

    //        public static double ANGLE_OFFSET = 0; // Degrees offset from center of target
    public static double ANGLE_OFFSET = Math.toRadians(8); // Degrees offset from center of target

    private static double THROTTLE_CONSTANT = 3; // Throttle constant for linear velocity

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    public PercentManualLineup() {
        super(P, I, D);
        requires(Robot.drivetrain);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.putNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR);
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KP", P);
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KI", I);
        SmartDashboard.putNumber("PercentLineup/ANGULAR_KD", D);
        SmartDashboard.putNumber("PercentLineup/MAX_VEL_THETA", MAX);
        SmartDashboard.putNumber("PercentLineup/MIN_VEL_THETA", MIN);
        SmartDashboard.putNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE);
        SmartDashboard.putNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        OUTPUT_SCALAR = SmartDashboard.getNumber("PercentLineup/OUTPUT_SCALAR", OUTPUT_SCALAR);
        P = SmartDashboard.getNumber("PercentLineup/ANGULAR_KP", P);
        I = SmartDashboard.getNumber("PercentLineup/ANGULAR_KI", I);
        D = SmartDashboard.getNumber("PercentLineup/ANGULAR_KD", D);
        MAX = SmartDashboard.getNumber("PercentLineup/MAX_VEL_THETA", MAX);
        MIN = SmartDashboard.getNumber("PercentLineup/MIN_VEL_THETA", MIN);
        DEADZONE = SmartDashboard.getNumber("PercentLineup/DEADZONE_VEL_THETA", DEADZONE);
        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineup/ANGLE_OFFSET", ANGLE_OFFSET);

        this.getPIDController().setP(P);
        this.getPIDController().setI(I);
        this.getPIDController().setD(D);

        logger.debug(String.format("Initialized with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f", P, I, D, MAX, MIN, DEADZONE));
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
        twist2DInput.setTwist(new Twist2D(OI.getPointDriveThrottle() * THROTTLE_CONSTANT, 0, -cmdVelTheta)); // TODO: Figure out why cmdVelTheta is negated
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
