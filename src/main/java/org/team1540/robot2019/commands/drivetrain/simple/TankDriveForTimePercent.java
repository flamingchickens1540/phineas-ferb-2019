package org.team1540.robot2019.commands.drivetrain.simple;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.datastructures.twod.Twist2D;

public class TankDriveForTimePercent extends TimedCommand { // TODO: Rooster

    private final double leftPercent;
    private final double rightPercent;
    private final boolean coast;

    public TankDriveForTimePercent(double timeoutSecs, double percent) {
        this(timeoutSecs, percent, false);
    }

    public TankDriveForTimePercent(double timeoutSecs, double percent, boolean coast) {
        this(timeoutSecs, percent, percent, coast);
    }

    public TankDriveForTimePercent(double timeoutSecs, double leftPercent, double rightPercent) {
        this(timeoutSecs, leftPercent, rightPercent, false);
    }

    public TankDriveForTimePercent(double timeoutSecs, double leftPercent, double rightPercent, boolean coast) {
        super(timeoutSecs);
        this.leftPercent = leftPercent;
        this.rightPercent = rightPercent;
        this.coast = coast;
        requires(Robot.drivetrain);
    }

    @Override
    protected void execute() {
        Robot.drivetrain.setLeftPercent(leftPercent);
        Robot.drivetrain.setRightPercent(rightPercent);
    }

    @Override
    protected void end() {
        if (!coast) {
            Robot.drivetrain.stop();
        } else {
            Robot.drivetrain.setPercentTwist(Twist2D.ZERO);
        }
    }
}
