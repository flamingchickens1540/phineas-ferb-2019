package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.auto.PercentManualLineupLocalizationAngleProvider;
import org.team1540.robot2019.commands.auto.PointControlConfig;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public class AutoLineupAndDrive extends PIDCommand { // TODO: Make this generic

    private static final Logger logger = Logger.getLogger(AutoLineupAndDrive.class);
    private static double CMD_VEL_X_SCALAR = 2.5;
    private static double MAX_VEL_X = 3;
    private static double MIN_VEL_X = 0.7;
    private static double OFFSET_X = 0.5;

    private static double CMD_VEL_X_MAX_ACCEL_UP = 0.05; // TODO: Use time

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    private double cmdVelScalar;
    private double max;
    private double min;
    private double deadzone;

    private boolean isConfigSet = false;
    private boolean endFlag = false;

    private PercentManualLineupLocalizationAngleProvider lineupLocalization = new PercentManualLineupLocalizationAngleProvider(Robot.odometry, Robot.deepSpaceVisionTargetLocalization);

    private boolean tempDisableLineup = false;
    private double cmdVelX;

    public AutoLineupAndDrive() {
        super(0, 0, 0);
        requires(Robot.drivetrain);

        this.setTimeout(4);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        SmartDashboard.putNumber("AutoLineupAndDrive/MIN_VEL_X", MIN_VEL_X);
        SmartDashboard.putNumber("AutoLineupAndDrive/MAX_VEL_X", MAX_VEL_X);
        SmartDashboard.putNumber("AutoLineupAndDrive/OFFSET_X", OFFSET_X);
        SmartDashboard.putNumber("AutoLineupAndDrive/CMD_VEL_X_SCALAR", CMD_VEL_X_SCALAR);
        SmartDashboard.putNumber("AutoLineupAndDrive/CMD_VEL_X_MAX_ACCEL_UP", CMD_VEL_X_MAX_ACCEL_UP);
    }

    @Override
    protected final void initialize() {
        endFlag = false;
        lineupLocalization.pointNextReset();
        lineupLocalization.initialize();
        applyConfig(lineupLocalization.getPointControlConfig());
        MIN_VEL_X = SmartDashboard.getNumber("AutoLineupAndDrive/MIN_VEL_X", MIN_VEL_X);
        MAX_VEL_X = SmartDashboard.getNumber("AutoLineupAndDrive/MAX_VEL_X", MAX_VEL_X);
        OFFSET_X = SmartDashboard.getNumber("AutoLineupAndDrive/OFFSET_X", OFFSET_X);
        CMD_VEL_X_SCALAR = SmartDashboard.getNumber("AutoLineupAndDrive/CMD_VEL_X_SCALAR", CMD_VEL_X_SCALAR);
        CMD_VEL_X_MAX_ACCEL_UP = SmartDashboard.getNumber("AutoLineupAndDrive/CMD_VEL_X_MAX_ACCEL_UP", CMD_VEL_X_MAX_ACCEL_UP);
    }

    public void applyConfig(PointControlConfig cfg) {
        this.cmdVelScalar = cfg.getOUTPUT_SCALAR();
        this.max = cfg.getMAX();
        this.min = cfg.getMIN();
        this.deadzone = cfg.getDEADZONE();

        this.getPIDController().setP(cfg.getP());
        this.getPIDController().setI(cfg.getI());
        this.getPIDController().setD(cfg.getD());

        this.isConfigSet = true;
    }

    public double getDistanceToVisionTarget() {
        return lineupLocalization.getDistanceToVisionTarget();
    }

    @Override
    protected final double returnPIDInput() {
        return -lineupLocalization.returnAngleError(); // returnPIDInput expects a position, so the error must be negated
    }

    @Override
    protected final void usePIDOutput(double output) {
        double desiredCmdVelX;
        double cmdVelTheta;
        if (isConfigSet && lineupLocalization.hasGoalBeenFound()) {
            cmdVelTheta = ControlUtils.allVelocityConstraints(output * cmdVelScalar, max, min, deadzone);
            desiredCmdVelX = ControlUtils.allVelocityConstraints(CMD_VEL_X_SCALAR * (lineupLocalization.getDistanceToVisionTarget() - OFFSET_X), MAX_VEL_X, MIN_VEL_X, 0);

        } else if (!lineupLocalization.hasGoalBeenFound()) {
            logger.debug("Goal not found! Setting vel to full.");
            cmdVelTheta = 0;
            desiredCmdVelX = MAX_VEL_X;
        } else {
            logger.warn("Config not set! Setting vel to zero.");
            cmdVelTheta = 0;
            desiredCmdVelX = 0;
        }
        if (desiredCmdVelX > cmdVelX) {
            cmdVelX += CMD_VEL_X_MAX_ACCEL_UP;
        }
        if (cmdVelX > desiredCmdVelX) {
            cmdVelX = desiredCmdVelX;
        }
        if (cmdVelX < MIN_VEL_X) {
            cmdVelX = MIN_VEL_X;
        }
        SmartDashboard.putNumber("AutoLineupAndDrive/CmdVelTheta", cmdVelTheta);
        twist2DInput.setTwist(new Twist2D(cmdVelX, 0, cmdVelTheta));
        pipeline.execute();
    }

    public void enableEndFlag() {
        endFlag = true;
    }

    @Override
    protected boolean isFinished() {
        return endFlag;
    }

    @Override
    protected final void end() {
        twist2DInput.setTwist(Twist2D.ZERO);
        pipeline.execute();
        logger.debug("Ended!");
    }
}
