package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;

public class TankDriveForTimeVelocity extends TimedCommand {

    private final double velocity;

    /**
     * @param timeoutSecs Seconds to drive
     * @param velocityMetersPerSecond Velocity in meters per second
     */
    public TankDriveForTimeVelocity(double timeoutSecs, double velocityMetersPerSecond) {
        super(timeoutSecs);
        this.velocity = velocityMetersPerSecond;
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.drivetrain.setLeftVelocityMetersPerSecond(velocity);
        Robot.drivetrain.setRightVelocityMetersPerSecond(velocity);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }
}
