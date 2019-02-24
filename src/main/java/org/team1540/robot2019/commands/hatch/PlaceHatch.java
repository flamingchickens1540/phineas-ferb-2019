package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class PlaceHatch extends CommandGroup {

    public PlaceHatch() {
        if (Robot.hatch.retracted()) {
            addSequential(new ExtendHatchMech());
            addSequential(new WaitCommand(Tuning.hatchExtendWaitTime));
        }
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(Tuning.hatchReleaseWaitTime));
        addSequential(new RetractHatchMech());
    }

}
