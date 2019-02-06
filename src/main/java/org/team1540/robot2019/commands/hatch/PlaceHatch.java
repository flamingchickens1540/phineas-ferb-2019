package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class PlaceHatch extends TimedCommand {

  public PlaceHatch() {
    super(Tuning.hatchPlaceTime);
    requires(Robot.hatchMech);
  }

  protected void initialize() {
    Robot.hatchMech.release();
  }

  protected void end() {
    Robot.hatchMech.slideIn();
  }

}
