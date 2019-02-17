package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.intake.IntakeAuto;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.rooster.util.SimpleCommand;

public class IntakeSequence extends CommandGroup {

  public IntakeSequence() {
    addParallel(new SimpleCommand("alaska's", Robot.hatchMech::slideIn, Robot.hatchMech));
    addSequential(new MoveElevatorToZero());
    addParallel(new IntakeAuto());
    addSequential(new LowerWrist());
  }
}
