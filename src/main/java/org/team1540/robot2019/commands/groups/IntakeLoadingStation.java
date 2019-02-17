package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.intake.IntakeAuto;
import org.team1540.rooster.util.SimpleCommand;

public class IntakeLoadingStation extends CommandGroup {

  public IntakeLoadingStation() {
    addParallel(new SimpleCommand("alaska's", Robot.hatchMech::slideIn, Robot.hatchMech));
    addSequential(new MoveElevatorToPosition(Tuning.elevatorLoadingStationPosition));
    addSequential(new IntakeAuto());
    addSequential(new MoveElevatorToZero());
  }
}
