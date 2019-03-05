package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatch;

public class PlaceHatch extends CommandGroup {

    public PlaceHatch() {
        if (Robot.hatch.retracted()) {
            addSequential(new ExtendHatch());
            addSequential(new WaitCommand(Tuning.hatchExtendWaitTime));
        }
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(Tuning.hatchReleaseWaitTime));
        addSequential(new RetractHatch());
    }

}
