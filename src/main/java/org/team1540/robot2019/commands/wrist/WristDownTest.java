package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class WristDownTest extends CommandGroup {

    public WristDownTest() {
        addSequential(new LowerWrist());
        addSequential(new TimedCommand(10000));
    }

}
