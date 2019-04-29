package org.team1540.robot2019.commands.drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Wiggle extends CommandGroup { // do do do, doo, doo, doo

    private static final double DRIVE_TIME = 0.3;
    private static final double SLOW_PERCENT = -0.05;
    private static final double FAST_PERCENT = 0.2;

    public static final double TOTAL_TIME = DRIVE_TIME * 3;

    public Wiggle() {
        addSequential(new TankDriveForTimePercent(DRIVE_TIME, SLOW_PERCENT, FAST_PERCENT));
        addSequential(new TankDriveForTimePercent(DRIVE_TIME * 2, FAST_PERCENT * 2, SLOW_PERCENT * 2));
    }
}
