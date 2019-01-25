package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class RaiseWrist extends Command {

  @Override
  protected void initialize() {
    Robot.wrist.moveUp();
  }

  @Override
  protected void end() {
    Robot.wrist.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.wrist.isAtTop();
  }
}
