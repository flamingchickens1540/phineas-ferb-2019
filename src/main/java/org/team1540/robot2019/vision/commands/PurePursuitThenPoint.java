package org.team1540.robot2019.vision.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;

public class PurePursuitThenPoint extends CommandGroup {

    public PurePursuitThenPoint() {
        addSequential(new PurePursuitLineup(Robot.limelightLocalization, Robot.wheelOdometry));

        addSequential(new PointLineupSimple());
//        addSequential(new PointLineup(Robot.limelightLocalization, Robot.wheelOdometry, Robot.lastOdomToVisionTarget));

//        addSequential(new DriveForTimeVelocity(0.4, 0.5));
    }

}
