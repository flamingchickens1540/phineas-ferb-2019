package org.team1540.robot2019.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class
IntakeAuto extends Command {

  public IntakeAuto() {
    super(Tuning.intakeTimeout);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    Robot.intake.startIntaking();
  }

  @Override
  protected void end() {
    Robot.intake.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.intake.hasBall() || isTimedOut();
  }
}
