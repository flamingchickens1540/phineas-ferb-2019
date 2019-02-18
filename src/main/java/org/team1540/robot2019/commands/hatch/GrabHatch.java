package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.util.SimpleCommand;

public class GrabHatch extends CommandGroup {

    public GrabHatch() {
        addSequential(new SimpleCommand("Hatch", Robot.hatch::grab, Robot.hatch));
        addSequential(new WaitCommand(Tuning.hatchGetTime));
        addSequential(new RetractHatch());
    }

}
