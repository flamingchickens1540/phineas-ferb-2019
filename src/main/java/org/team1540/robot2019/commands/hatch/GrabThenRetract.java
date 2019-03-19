package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class GrabThenRetract extends CommandGroup {

    public GrabThenRetract() {
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(Tuning.hatchGrabWaitTime));
        addSequential(new RetractHatchMech());
    }

}
