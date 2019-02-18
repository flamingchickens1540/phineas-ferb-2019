package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.rooster.util.SimpleCommand;

public class GetHatchFloor extends CommandGroup {

  public GetHatchFloor() {
    addSequential(new SimpleCommand("in", Robot.hatchMech::slideIn, Robot.hatchMech));
    addSequential(new SimpleCommand("close", Robot.hatchMech::detach, Robot.hatchMech));
    addSequential(new MoveElevatorToZero());
    addSequential(new LowerWrist());
    addSequential(new SimpleCommand("out", Robot.hatchMech::slideOut, Robot.hatchMech));
    addSequential(new Command() {
      @Override
      protected boolean isFinished() {
        return false;
      }
    });
  }
}
