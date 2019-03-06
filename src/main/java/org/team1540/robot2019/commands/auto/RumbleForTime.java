package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class RumbleForTime extends CommandGroup {

    public RumbleForTime(XboxController xboxController, double timeSeconds) {
        addSequential(new Rumble(xboxController, 1));
        addSequential(new TimedCommand(timeSeconds));
        addSequential(new Rumble(xboxController, 0));
    }
}
