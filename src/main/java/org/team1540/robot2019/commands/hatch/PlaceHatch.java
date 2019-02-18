package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.util.SimpleCommand;

public class PlaceHatch extends CommandGroup {

    public PlaceHatch() {
//    addSequential(new HatchSlideOut());
//    addSequential(new WaitCommand(0.3));
        // TODO: Extend
        addSequential(new SimpleCommand("Hatch Release", Robot.hatch::release, Robot.hatch));
        addSequential(new WaitCommand(Tuning.hatchPlaceTime));
        addSequential(new SimpleCommand("Hatch Slide In", Robot.hatch::retract, Robot.hatch));
    }

}
