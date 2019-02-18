package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;

public class GetHatch extends CommandGroup {

  public GetHatch() {
    addSequential(new HatchSlideOut());
//    addSequential(new WaitCommand(Tuning.hatchGetTime));
//    addSequential(new GrabHatch());
//    addSequential(new WaitCommand(0.3));
//    addSequential(new HatchSlideIn());
  }

}
