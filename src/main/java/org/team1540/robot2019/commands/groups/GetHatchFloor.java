package org.team1540.robot2019.commands.groups;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.HatchSlideIn;
import org.team1540.robot2019.commands.hatch.HatchSlideOut;
import org.team1540.robot2019.commands.wrist.LowerWrist;
import org.team1540.rooster.util.SimpleCommand;

public class GetHatchFloor extends CommandGroup {

  public GetHatchFloor() {
    addParallel(new HatchSlideIn());
    addParallel(new SimpleCommand("close", Robot.hatchMech::release, Robot.hatchMech));
    addSequential(new MoveElevatorToZero());
    addSequential(new LowerWrist());
    addSequential(new WaitCommand(Tuning.hatchFloorTime));
    addSequential(new HatchSlideOut());
    addSequential(new Command() {
      @Override
      protected boolean isFinished() {
        return false;
      }
    });
  }
}
