package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class GetHatch extends TimedCommand {

  public GetHatch() {
    super(Tuning.hatchGetTime);
    requires(Robot.hatchMech);
  }

  protected void initialize() {
    Robot.hatchMech.slideOut();
  }

  protected void end() {
    Robot.hatchMech.attach();
  }

}
