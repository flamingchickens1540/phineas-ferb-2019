package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class MoveElevatorUp extends Command {

  public MoveElevatorUp() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    if (!Robot.elevator.isAtTop()) {
      Robot.elevator.startMovingUp();
    }
  }

  @Override
  protected void end() {
    Robot.elevator.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.elevator.isAtTop();
  }
}
