package org.team1540.robot2019.commands.drivetrain.pointdrive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.MiniPID;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public abstract class PointManualDriveCommand extends Command {

    private static final Logger logger = Logger.getLogger(PointManualDriveCommand.class);
    public static double FAST = 3;
    public static double SLOW = 1;
    public static double FAST_X = 1.5;
    public static double SLOW_X = 1;

    public static double SPEED_FF = 0;

    private final MiniPID pointPID;

    public static double THROTTLE_P = 0.7;
    public static double THROTTLE_I = 0;
    public static double THROTTLE_D = 1;
    public static double THROTTLE_F = 0.33;

    private final MiniPID throttlePID;

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    private double outputScalar;
    private double max;
    private double min;
    private double deadzone;
    private double throttleConstant;

    private boolean isConfigSet = false;

    public PointManualDriveCommand() {
        pointPID = new MiniPID(0, 0, 0);
        pointPID.setOutputLimits(1);
        requires(Robot.drivetrain);

        SmartDashboard.putNumber("PointManualDrive/SPEED_FF", SPEED_FF);
        SmartDashboard.putNumber("PointManualDrive/THROTTLE_P", THROTTLE_P);
        SmartDashboard.putNumber("PointManualDrive/THROTTLE_I", THROTTLE_I);
        SmartDashboard.putNumber("PointManualDrive/THROTTLE_D", THROTTLE_D);
        SmartDashboard.putNumber("PointManualDrive/THROTTLE_F", THROTTLE_F);

        SmartDashboard.putNumber("PointManualDrive/FAST", FAST);
        SmartDashboard.putNumber("PointManualDrive/SLOW", SLOW);
        SmartDashboard.putNumber("PointManualDrive/FAST_X", FAST_X);
        SmartDashboard.putNumber("PointManualDrive/SLOW_X", SLOW_X);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        throttlePID = new MiniPID(0, 0, 0);
        throttlePID.setOutputLimits(0, 1); // can only slow down
    }

    @Override
    protected final void initialize() {
        SPEED_FF = SmartDashboard.getNumber("PointManualDrive/SPEED_FF", SPEED_FF);
        THROTTLE_P = SmartDashboard.getNumber("PointManualDrive/THROTTLE_P", THROTTLE_P);
        THROTTLE_I = SmartDashboard.getNumber("PointManualDrive/THROTTLE_I", THROTTLE_I);
        THROTTLE_D = SmartDashboard.getNumber("PointManualDrive/THROTTLE_D", THROTTLE_D);
        THROTTLE_F = SmartDashboard.getNumber("PointManualDrive/THROTTLE_F", THROTTLE_F);

        FAST = SmartDashboard.getNumber("PointManualDrive/FAST", FAST);
        SLOW = SmartDashboard.getNumber("PointManualDrive/SLOW", SLOW);
        FAST_X = SmartDashboard.getNumber("PointManualDrive/FAST_X", FAST_X);
        SLOW_X = SmartDashboard.getNumber("PointManualDrive/SLOW_X", SLOW_X);
        logger.debug("Initialized with SPEED_FF: " + SPEED_FF);
        applyConfig(initializeAndGetConfig());
    }

    protected abstract PointControlConfig initializeAndGetConfig();

    public void applyConfig(PointControlConfig cfg) {
        this.outputScalar = cfg.getOUTPUT_SCALAR();
        this.max = cfg.getMAX();
        this.min = cfg.getMIN();
        this.deadzone = cfg.getDEADZONE();
        this.throttleConstant = cfg.getTHROTTLE_CONSTANT();

        pointPID.setPID(cfg.getP(), cfg.getI(), cfg.getD());
        throttlePID.setPID(THROTTLE_P, THROTTLE_I, THROTTLE_D, THROTTLE_F);

        this.isConfigSet = true;


        logger.debug(String
            .format("Applied config with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f OutputScalar:%f throtConstant:%f", cfg.getP(), cfg.getI(), cfg.getD(), max, min, deadzone, outputScalar,
                throttleConstant));
    }

    @Override
    protected void execute() {
        if (isConfigSet) {
            double output = pointPID.getOutput(-returnAngleError());
            double cmdVelTheta = ControlUtils.allVelocityConstraints(output * outputScalar, max, min, deadzone);
            SmartDashboard.putNumber("PointManualDrive/CmdVelTheta", cmdVelTheta);
            double outputPercent = OI.getPointDriveThrottle();
            if (Robot.drivetrain.getDriveCommand().isLineupRunning()) {
                double targetXVel = ControlUtils.linearDeadzoneRamp(Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget(), false, FAST, SLOW, FAST_X, SLOW_X);
//                double targetXVel = 2;
                double pidOutput = throttlePID.getOutput(Robot.drivetrain.getTwist().getX(), targetXVel);
                SmartDashboard.putNumber("PointManualDrive/throttlePIDError", targetXVel - Robot.drivetrain.getTwist().getX());
                if (outputPercent > 0) {
                    outputPercent *= pidOutput;
                }
            }
            twist2DInput.setTwist(new Twist2D(outputPercent * throttleConstant, 0, cmdVelTheta));
            SmartDashboard.putNumber("PointManualDrive/velX", Robot.drivetrain.getTwist().getX());
        } else {
            logger.warn("Config not set! Setting vel to zero.");
            twist2DInput.setTwist(Twist2D.ZERO);
        }
        pipeline.execute();
    }
//
//    @Override
//    protected final void usePIDOutput(double output) {
//
//    }
//
//    @Override
//    protected final double returnPIDInput() {
//        double angleError = -returnAngleError();
//        SmartDashboard.putNumber("PointManualDrive/AngleError", -angleError);
//        return angleError; // returnPIDInput expects a position, so the error must be negated
//    }

    protected abstract double returnAngleError(); // TODO: This should return the desired position, not error
}
