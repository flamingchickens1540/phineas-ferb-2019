package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.groups.PrepareForClimb;

public class Level3Climb extends CommandGroup {

    public Level3Climb() {
        addSequential(new PrepareForClimb());
        addSequential(new RaiseUpGyroAssist());
    }

}
