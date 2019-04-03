package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.hatch.AutoGrabHatchSequence;

public class VisionGrabSequence extends CommandGroup {

    public VisionGrabSequence() {
        addSequential(new WaitUntilCommand(() -> {
            return Robot.drivetrain.getDriveCommand().getLineupLocalization().getDistanceToVisionTarget() < 0.35
                && Robot.drivetrain.getDriveCommand().getLineupLocalization().returnAngleError() < Math.toRadians(3);
        }));
        addSequential(new AutoGrabHatchSequence());
    }
}
