package org.team1540.robot2019.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class IntakeAuto extends Command {

  private static final Logger logger = Logger.getLogger(IntakeAuto.class);

  public IntakeAuto() {
//    super(Tuning.intakeTimeout);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
    logger.debug("Intake starting");
    Robot.intake.startIntaking();
  }

  @Override
  protected void end() {
    if (isTimedOut()) {
      logger.debug("Intake stopping due to timeout");
    } else {
      logger.debug("Intake stopping due to detected ball");
    }
    Robot.intake.stop();
  }

  @Override
  protected boolean isFinished() {
    return Robot.intake.hasBall() || isTimedOut();
  }
}
