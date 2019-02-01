package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class ZeroElevator extends Command {

  private static final Logger logger = Logger.getLogger(ZeroElevator.class);

  public ZeroElevator() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    logger.debug("Zeroing elevator");

    Robot.elevator.setRaw(-Tuning.elevatorZeroingThrottle);
  }

  @Override
  protected void end() {
    logger.debug(
        "Re-zeroed elevator (previously read position was " + Robot.elevator.getPosition() + ")");

    Robot.elevator.zero();
    Robot.elevator.setRaw(0);
  }

  @Override
  protected void interrupted() {
    logger.debug("Elevator zeroing interrupted");

    Robot.elevator.setRaw(0);
  }

  @Override
  protected boolean isFinished() {
    return Robot.elevator.isAtLimit();
  }
}
