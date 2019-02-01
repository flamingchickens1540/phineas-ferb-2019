package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.intake.IntakeAuto;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.robot2019.commands.wrist.RaiseWrist;

public class IntakeSequence extends CommandGroup {

  public IntakeSequence() {
    addSequential(new MoveElevatorToZero());
    addSequential(new LowerWrist());
    addSequential(new IntakeAuto());
    addSequential(new RaiseWrist());
  }
}
