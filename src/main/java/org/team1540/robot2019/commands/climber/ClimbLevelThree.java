package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbLevelThree extends CommandGroup {

    public ClimbLevelThree() {
        addSequential(new PrepareForClimb());
        addSequential(new ExtendGyroStabilize());
    }

}
