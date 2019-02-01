package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MoveElevatorToZero extends CommandGroup {

  public MoveElevatorToZero() {
    addSequential(new MoveElevatorToPosition(0));
    addSequential(new ZeroElevator());
  }
}
