package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.GetHatch;
import org.team1540.robot2019.commands.hatch.PlaceHatch;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.commands.wrist.RaiseWrist;

public class GetHatchFloor extends CommandGroup {

  public GetHatchFloor() {
    addSequential(new MoveElevatorToZero());
    addSequential(new LowerWrist());
    addSequential(new GetHatch());
    addSequential(new RaiseWrist());
  }

}
