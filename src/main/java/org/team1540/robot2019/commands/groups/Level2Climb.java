package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.climber.MoveArmsToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.rooster.util.SimpleCommand;

public class Level2Climb extends CommandGroup {

  public Level2Climb() {
    addSequential(new MoveElevatorToPosition(Tuning.elevatorClimbPosition));
    addSequential(new MoveArmsToPosition(55000));
  }
}
