package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class MoveElevatorDown extends Command {

  public MoveElevatorDown() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    if (!Robot.elevator.isAtBottom()) {
      Robot.elevator.startMovingDown();
    }
  }

  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.elevator.isAtBottom();
  }
}
