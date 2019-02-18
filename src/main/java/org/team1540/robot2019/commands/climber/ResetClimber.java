package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class ResetClimber extends CommandGroup {

  public ResetClimber() {
    addSequential(new MoveArmsToPosition(Tuning.climberBackPos));
    addSequential(new MoveElevatorToZero());
  }

}
