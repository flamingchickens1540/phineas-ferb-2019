package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.util.SimpleCommand;

public class PlaceHatch extends CommandGroup {

  public PlaceHatch() {
//    addSequential(new ExtendHatch());
//    addSequential(new WaitCommand(Tuning.hatchPlaceTime1));
    addSequential(new SimpleCommand("release", Robot.hatch::release, Robot.hatch));
    addSequential(new WaitCommand(Tuning.hatchPlaceTime2));
    addSequential(new RetractHatch());
  }

}
