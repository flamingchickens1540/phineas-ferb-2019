package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;

public class TankDriveForTimePercent extends TimedCommand {

    private final double leftPercent;
    private final double rightPercent;

    public TankDriveForTimePercent(double timeoutSecs, double percent) {
        super(timeoutSecs);
        this.leftPercent = this.rightPercent = percent;
        requires(Robot.drivetrain);
    }

    public TankDriveForTimePercent(double timeoutSecs, double leftPercent, double rightPercent) {
        super(timeoutSecs);
        this.leftPercent = leftPercent;
        this.rightPercent = rightPercent;
        requires(Robot.drivetrain);
    }

    @Override
    protected void execute() {
        Robot.drivetrain.setLeftPercent(leftPercent);
        Robot.drivetrain.setRightPercent(rightPercent);
    }

    @Override
    protected void end() {
        Robot.drivetrain.stop();
    }
}
