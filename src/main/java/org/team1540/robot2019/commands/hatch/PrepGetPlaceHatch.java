package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class PrepGetPlaceHatch extends CommandGroup {

    public PrepGetPlaceHatch() {
        addSequential(new SimpleCommand("Hatch Slide Out", Robot.hatch::extend, Robot.hatch));
//    addSequential(new GrabHatch());
//    addSequential(new WaitCommand(0.3));
//    addSequential(new HatchSlideIn());
    }

}
