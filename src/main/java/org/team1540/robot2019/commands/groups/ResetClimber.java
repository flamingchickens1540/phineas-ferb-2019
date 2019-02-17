package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.climber.MoveArmsToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class ResetClimber extends CommandGroup {

  public ResetClimber() {
    addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
    addSequential(new MoveArmsToPosition(Tuning.climberBackPos));
    addSequential(new MoveElevatorToZero());
  }

}
