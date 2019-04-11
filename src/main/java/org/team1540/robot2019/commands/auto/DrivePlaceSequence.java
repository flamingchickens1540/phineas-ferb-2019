package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.drivetrain.AutoLineupAndDrive;
import org.team1540.robot2019.commands.hatch.PlaceHatchSequence;

public class DrivePlaceSequence extends CommandGroup {

    public DrivePlaceSequence() {
        AutoLineupAndDrive autoLineupAndDrive = new AutoLineupAndDrive();
        addParallel(new WaitUntilCommand(() -> autoLineupAndDrive.getDistanceToVisionTarget() < 0.5, autoLineupAndDrive::enableEndFlag));
        addSequential(autoLineupAndDrive);
        addSequential(new PlaceHatchSequence(true, true));
    }
}
