package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.climber.MoveArmsToStart;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;

public class PrepareForClimb extends CommandGroup {

  public PrepareForClimb() {
    addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
    addSequential(new MoveArmsToStart());
  }
}
