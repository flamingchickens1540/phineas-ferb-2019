package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.PlaceHatch;
import org.team1540.robot2019.commands.intake.Eject;

public class PlaceHatchThenDown extends CommandGroup {

  public PlaceHatchThenDown() {
    addSequential(new PlaceHatch());
    addSequential(new MoveElevatorToZero());
  }

}
