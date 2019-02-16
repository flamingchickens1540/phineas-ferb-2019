package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.climber.MoveArmsToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.rooster.util.SimpleCommand;

public class ResetClimber extends CommandGroup {

  public ResetClimber() {
    addSequential(new MoveArmsToPosition(Tuning.climberBackPos));
    addSequential(new MoveElevatorToZero());
  }

}
