package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.commands.wrist.RaiseWrist;
import org.team1540.robot2019.commands.intake.IntakeAuto;

public class IntakeSequence extends CommandGroup {

  public IntakeSequence() {
    addSequential(new MoveElevatorToPosition(Tuning.elevatorDownPosition));
    addSequential(new LowerWrist());
    addSequential(new IntakeAuto());
    addSequential(new RaiseWrist());
  }
}
