package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.PIDCommand;
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

    private Executable pipeline;
    private TankDriveTwist2DInput twist2DInput;

    private double outputScalar;
    private double max;
    private double min;
    private double deadzone;
    private double throttleConstant;

    public PointManualDriveCommand(double p, double i, double d, double outputScalar, double max, double min, double deadzone, double throttleConstant) {
        super(p, i, d);
        this.outputScalar = outputScalar;
        this.max = max;
        this.min = min;
        this.deadzone = deadzone;
        this.throttleConstant = throttleConstant;
        requires(Robot.drivetrain);

        twist2DInput = new TankDriveTwist2DInput(Tuning.drivetrainRadiusMeters);
        pipeline = twist2DInput
            .then(new FeedForwardProcessor(Tuning.driveKV, Tuning.driveVIntercept, 0))
            .then(new UnitScaler(Tuning.drivetrainTicksPerMeter, 10))
            .then(Robot.drivetrain.getPipelineOutput(false));
    }

    @Override
    protected final void usePIDOutput(double output) {
        double cmdVelTheta = ControlUtils.allVelocityConstraints(output*outputScalar, max, min, deadzone);
        twist2DInput.setTwist(new Twist2D(OI.getPointDriveThrottle() * throttleConstant, 0, -cmdVelTheta)); // TODO: Figure out why cmdVelTheta is negated
        pipeline.execute();
    }

    @Override
    protected final double returnPIDInput() {
        return returnAngleError();
    }

    protected abstract double returnAngleError();
}
