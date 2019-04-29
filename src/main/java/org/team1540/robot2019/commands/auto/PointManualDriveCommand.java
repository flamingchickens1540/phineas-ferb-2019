package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.apache.log4j.Logger;
import org.team1540.robot2019.OI;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.datastructures.twod.Twist2D;
import org.team1540.robot2019.utils.ControlUtils;
import org.team1540.robot2019.utils.TankDriveTwist2DInput;
import org.team1540.rooster.drive.pipeline.FeedForwardProcessor;
import org.team1540.rooster.drive.pipeline.UnitScaler;
import org.team1540.rooster.functional.Executable;

public abstract class PointManualDriveCommand extends PIDCommand {

    private static final Logger logger = Logger.getLogger(PointManualDriveCommand.class);

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    private double outputScalar;
    private double max;
    private double min;
    private double deadzone;
    private double throttleConstant;

    private boolean isConfigSet = false;

    public PointManualDriveCommand() {
        super(0, 0, 0);
        requires(Robot.drivetrain);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));
    }

    @Override
    protected final void initialize() {
        applyConfig(initializeAndGetConfig());
    }

    protected abstract PointControlConfig initializeAndGetConfig();

    public void applyConfig(PointControlConfig cfg) {
        this.outputScalar = cfg.getOUTPUT_SCALAR();
        this.max = cfg.getMAX();
        this.min = cfg.getMIN();
        this.deadzone = cfg.getDEADZONE();
        this.throttleConstant = cfg.getTHROTTLE_CONSTANT();

        this.getPIDController().setP(cfg.getP());
        this.getPIDController().setI(cfg.getI());
        this.getPIDController().setD(cfg.getD());

        this.isConfigSet = true;

        logger.debug(String
            .format("Applied config with P:%f I:%f D:%f Max:%f Min:%f Deadzone:%f OutputScalar:%f throtConstant:%f", cfg.getP(), cfg.getI(), cfg.getD(), max, min, deadzone, outputScalar,
                throttleConstant));
    }

    @Override
    protected final void usePIDOutput(double output) {
        if (isConfigSet) {
            double cmdVelTheta = ControlUtils.allVelocityConstraints(output * outputScalar, max, min, deadzone);
            SmartDashboard.putNumber("PointManualDrive/CmdVelTheta", cmdVelTheta);
            twist2DInput.setTwist(new Twist2D(OI.getPointDriveThrottle() * throttleConstant, 0, cmdVelTheta));
        } else {
            logger.warn("Config not set! Setting vel to zero.");
            twist2DInput.setTwist(Twist2D.ZERO);
        }
        pipeline.execute();
    }

    @Override
    protected final double returnPIDInput() {
        return -returnAngleError(); // returnPIDInput expects a position, so the error must be negated
    }

    protected abstract double returnAngleError();
}
