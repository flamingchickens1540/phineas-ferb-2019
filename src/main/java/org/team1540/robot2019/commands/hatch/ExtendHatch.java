package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class ExtendHatch extends CommandGroup {

    public ExtendHatch() { // Note: this is a command group for future use
        addSequential(new SimpleCommand("HatchMech Slide Out", Robot.hatch::extend, Robot.hatch));
//    addSequential(new GrabHatch());
//    addSequential(new WaitCommand(0.3));
//    addSequential(new HatchSlideIn());
    }

}
