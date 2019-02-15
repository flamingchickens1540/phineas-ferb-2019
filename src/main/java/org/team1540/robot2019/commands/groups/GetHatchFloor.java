package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GetHatch;
import org.team1540.robot2019.commands.hatch.PlaceHatch;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.rooster.util.SimpleCommand;

public class GetHatchFloor extends CommandGroup {

  public GetHatchFloor() {
    addSequential(new SimpleCommand("in", Robot.hatchMech::slideIn, Robot.hatchMech));
    addSequential(new SimpleCommand("close", Robot.hatchMech::release, Robot.hatchMech));
    addSequential(new MoveElevatorToZero());
    addSequential(new WaitCommand(Tuning.hatchFloorTime));
    addSequential(new LowerWrist());
    addSequential(new GetHatch());
  }

}
