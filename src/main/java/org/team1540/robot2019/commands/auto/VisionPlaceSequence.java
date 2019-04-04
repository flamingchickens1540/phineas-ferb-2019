package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.hatch.AutoPlaceHatchSequenceNoDTReq;

public class VisionPlaceSequence extends CommandGroup {

    public VisionPlaceSequence() {
        addSequential(new WaitUntilCommand(() -> {
            return Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget() < 0.4
                && Robot.drivetrain.getDriveCommand().getLineupLocalization().returnAngleError(0) < Math.toRadians(3);
        }));
        addSequential(new AutoPlaceHatchSequenceNoDTReq());
    }
}
