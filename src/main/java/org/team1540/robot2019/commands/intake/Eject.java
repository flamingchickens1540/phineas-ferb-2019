package org.team1540.robot2019.commands.intake;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class Eject extends TimedCommand {

  public Eject() {
    super(Tuning.intakeEjectTime);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    Robot.intake.startEjecting();
  }

  @Override
  protected void end() {
    Robot.intake.stop();
  }
}
