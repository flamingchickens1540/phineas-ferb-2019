package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class GrabHatch extends CommandGroup {

    public GrabHatch() {
        // TODO: Slide out
        addSequential(new SimpleCommand("HatchMech Grab", Robot.hatch::grab, Robot.hatch));
        // TODO: Wait and slide in
//    addSequential(new GrabHatch());
//    addSequential(new WaitCommand(0.3));
//    addSequential(new HatchSlideIn());
    }

}
