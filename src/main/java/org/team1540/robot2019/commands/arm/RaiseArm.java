package org.team1540.robot2019.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class RaiseArm extends Command {

  @Override
  protected void initialize() {
    Robot.arm.moveUp();
  }

  @Override
  protected void end() {
    Robot.arm.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.arm.isAtTop();
  }
}
