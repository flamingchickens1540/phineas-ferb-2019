package org.team1540.robot2019.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class MoveArm extends InstantCommand {

  private boolean raise;

  public MoveArm(boolean raise) {
    requires(Robot.arm);
    this.raise = raise;
  }

  @Override
  protected void execute() {
    if (raise) {
      Robot.arm.moveUp();
    } else {
      Robot.arm.moveDown();
    }
  }
}
