package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.hatch.AutoPlaceHatchSequenceNoDTReq;

public class VisionPlaceSequence extends CommandGroup {

    public static final double MAX_DISTANCE = 0.4;
    public static final double MAX_ANGLE_ERROR = Math.toRadians(3);

    public VisionPlaceSequence() {
        addSequential(new WaitUntilCommand(() ->
            Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget() < MAX_DISTANCE
                && Robot.drivetrain.getDriveCommand().getLineupLocalization().returnAngleError(0) < MAX_ANGLE_ERROR));
        addSequential(new AutoPlaceHatchSequenceNoDTReq());
    }
}
