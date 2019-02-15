package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.climber.MoveArmsToStart;

public class ResetClimber extends CommandGroup {

  public ResetClimber() {
    addSequential(new MoveArmsToStart());
//    addSequential(new MoveElevatorToZero());
  }

}
