package org.team1540.robot2019.commands.hatch.subgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class StowHatchMech extends CommandGroup {
    public StowHatchMech() {
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
    }
}
