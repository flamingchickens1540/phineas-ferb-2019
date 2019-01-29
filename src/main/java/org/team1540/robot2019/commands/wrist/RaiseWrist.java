package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.subsystems.Wrist.WristState;

public class RaiseWrist extends Command {

  @Override
  protected void initialize() {
    Robot.wrist.moveUp();
  }

  @Override
  protected boolean isFinished() {
    return Robot.wrist.getState() == WristState.OFF_UP;
  }
}
