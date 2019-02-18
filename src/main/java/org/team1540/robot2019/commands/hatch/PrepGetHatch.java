package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class PrepGetHatch extends CommandGroup {

    public PrepGetHatch() {
        addSequential(new SimpleCommand("Hatch Slide Out", Robot.hatch::slideOut, Robot.hatch));
//    addSequential(new GrabHatch());
//    addSequential(new WaitCommand(0.3));
//    addSequential(new HatchSlideIn());
  }

}
