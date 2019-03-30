package org.team1540.robot2019.commands.rumble;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class RumbleForTime extends CommandGroup {

    public RumbleForTime(XboxController xboxController, double timeSeconds) {
        addSequential(new SetRumble(xboxController, 1));
        addSequential(new TimedCommand(timeSeconds));
        addSequential(new SetRumble(xboxController, 0));
    }
}
