package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;

public class DriveVelocityForTime extends TimedCommand {

    private final double velocity;

    public DriveVelocityForTime(double timeoutSecs, double velocity) {
        super(timeoutSecs);
        this.velocity = velocity;
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.drivetrain.setLeftVelocityTPU(velocity);
        Robot.drivetrain.setRightVelocityTPU(velocity);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }
}
