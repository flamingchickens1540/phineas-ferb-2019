package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.intake.Eject;

public class EjectThenDown extends CommandGroup {

  public EjectThenDown() {
    addSequential(new Eject());
    addSequential(new MoveElevatorToZero());
  }

}
