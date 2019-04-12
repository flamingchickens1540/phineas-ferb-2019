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
    public static double SPEED_FF = 0;

    private final MiniPID pointPID;

    public static double THROTTLE_P = 0.25;
    public static double THROTTLE_I = 0;
    public static double THROTTLE_D = 0;
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

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));

        throttlePID = new MiniPID(0, 0, 0);
        throttlePID.setOutputLimits(-1, 0); // can only slow down
    }

    @Override
    protected final void initialize() {
        SPEED_FF = SmartDashboard.getNumber("PointManualDrive/SPEED_FF", SPEED_FF);
        THROTTLE_P = SmartDashboard.getNumber("PointManualDrive/THROTTLE_P", THROTTLE_P);
        THROTTLE_I = SmartDashboard.getNumber("PointManualDrive/THROTTLE_I", THROTTLE_I);
        THROTTLE_D = SmartDashboard.getNumber("PointManualDrive/THROTTLE_D", THROTTLE_D);
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
        throttlePID.setPID(THROTTLE_P, THROTTLE_I, THROTTLE_D);

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
                double targetXVel = ControlUtils.linearDeadzoneRamp(Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget(), false, 3, 1, 1.5, 1);
                double pidOutput = throttlePID.getOutput(Robot.drivetrain.getTwist().getX(), targetXVel);
                SmartDashboard.putNumber("PointManualDrive/throttlePIDOutput", pidOutput);
                outputPercent += pidOutput;
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
