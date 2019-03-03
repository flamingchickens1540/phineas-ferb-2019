package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;

public class GrabHatchThenBack extends CommandGroup {

    public GrabHatchThenBack() {
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(Tuning.hatchGrabWaitTime));
        addSequential(new RetractHatchMech());
    }

}
