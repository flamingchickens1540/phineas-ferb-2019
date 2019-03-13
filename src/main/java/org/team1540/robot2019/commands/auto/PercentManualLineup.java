package org.team1540.robot2019.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class PercentManualLineup extends PIDCommand {

    public static final Logger logger = Logger.getLogger(PercentManualLineup.class);

    // Max/Min angular velocity
    public static double MIN_VEL_THETA = 0.06;
    public static double MAX_VEL_THETA = 1;

    public static double DEADZONE_VEL_THETA = 0.005;

    // Constants for angular VPID controller
    public static double ANGULAR_KP = -0.55;
    public static double ANGULAR_KI = 0;
    public static double ANGULAR_KD = -2;

    public static double ANGLE_OFFSET = Math.toRadians(5.5); // Degrees offset from center of target

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;
    private Double goal = null;

    public PercentManualLineup() {
        super(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD);
        requires(Robot.drivetrain);
        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(0, 0, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput());

        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KP", PercentManualLineup.ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KI", PercentManualLineup.ANGULAR_KI);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGULAR_KD", PercentManualLineup.ANGULAR_KD);
        SmartDashboard.setDefaultNumber("PercentLineup/MIN_VEL_THETA", PercentManualLineup.MIN_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/DEADZONE_VEL_THETA", PercentManualLineup.DEADZONE_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/MAX_VEL_THETA", PercentManualLineup.MAX_VEL_THETA);
        SmartDashboard.setDefaultNumber("PercentLineup/ANGLE_OFFSET", PercentManualLineup.ANGLE_OFFSET);
    }

    @Override
    protected void initialize() {
        ANGULAR_KP = SmartDashboard.getNumber("PercentLineup/ANGULAR_KP", PercentManualLineup.ANGULAR_KP); // TODO: Remove temporary tuning (yaml ftw)
        ANGULAR_KI = SmartDashboard.getNumber("PercentLineup/ANGULAR_KI", PercentManualLineup.ANGULAR_KI);
        ANGULAR_KD = SmartDashboard.getNumber("PercentLineup/ANGULAR_KD", PercentManualLineup.ANGULAR_KD);
        MIN_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MIN_VEL_THETA", PercentManualLineup.MIN_VEL_THETA);
        DEADZONE_VEL_THETA = SmartDashboard.getNumber("PercentLineup/DEADZONE_VEL_THETA", PercentManualLineup.DEADZONE_VEL_THETA);
        MAX_VEL_THETA = SmartDashboard.getNumber("PercentLineup/MAX_VEL_THETA", PercentManualLineup.MAX_VEL_THETA);
        ANGLE_OFFSET = SmartDashboard.getNumber("PercentLineup/ANGLE_OFFSET", PercentManualLineup.ANGLE_OFFSET);

        this.getPIDController().setP(ANGULAR_KP);
        this.getPIDController().setI(ANGULAR_KI);
        this.getPIDController().setD(ANGULAR_KD);
        System.out.printf("Config updated: P: %f I: %f D: %f Max: %f Min: %f", ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, MAX_VEL_THETA, MIN_VEL_THETA);
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
        return false;
    }

    @Override
    protected void end() {
        logger.debug("SimplePointToTarget Ended!");
    }

    private double getAngleError(double x) {
        double error = TrigUtils.signedAngleError(x, Hardware.navx.getYawRadians());
        SmartDashboard.putNumber("PercentLineup/AngleError", error);
        return error;
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
//        output *= MAX_VEL_THETA;
        double cmdVelTheta = ControlUtils.velocityPosNegConstrain(output, MAX_VEL_THETA, MIN_VEL_THETA);
        if (Math.abs(output) < DEADZONE_VEL_THETA || Robot.elevator.getPosition() > 4) {
            cmdVelTheta = 0;
        }
        Twist2D cmdVel = new Twist2D(OI.getTankdriveLeftAxis() * -0.7, 0, cmdVelTheta);
        Robot.drivetrain.setPercentTwist(cmdVel);
//        twist2DInput.setTwist(cmdVel);
//        pipeline.execute();
    }
}
