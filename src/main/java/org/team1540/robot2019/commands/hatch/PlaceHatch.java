package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;

public class PlaceHatch extends CommandGroup {

  public PlaceHatch() {
//    addSequential(new HatchSlideOut());
//    addSequential(new WaitCommand(0.3));
    addSequential(new ReleaseHatch());
    addSequential(new WaitCommand(Tuning.hatchPlaceTime));
    addSequential(new HatchSlideIn());
  }

}
