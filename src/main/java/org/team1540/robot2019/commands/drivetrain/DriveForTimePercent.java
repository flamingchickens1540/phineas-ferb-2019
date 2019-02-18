package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;

public class DriveForTimePercent extends TimedCommand {

    private final double percent;

    public DriveForTimePercent(double timeoutSecs, double percent) {
        super(timeoutSecs);
        this.percent = percent;
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.drivetrain.setLeftPercent(percent);
        Robot.drivetrain.setRightPercent(percent);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }
}
