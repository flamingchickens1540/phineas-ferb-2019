package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;
import org.team1540.robot2019.utils.WaitUntilCommand;

public class DrivePlaceHatchSequence extends CommandGroup {

    public DrivePlaceHatchSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new WaitUntilCommand(() -> autoLineupAndDrive.getDistanceToVisionTarget() < 0.5, autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new PlaceHatchSequence(true, true));
    }
}
