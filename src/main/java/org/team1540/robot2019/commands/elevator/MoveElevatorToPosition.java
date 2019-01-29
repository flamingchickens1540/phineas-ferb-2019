package org.team1540.robot2019.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class MoveElevatorToPosition extends Command {

  private static final Logger logger = Logger.getLogger(MoveElevatorToPosition.class);

  double position;

    public MoveElevatorToPosition(double pos) {
      this.position = pos;
    }

    @Override
    protected void initialize() {
      logger.debug("Moving elevator to position " + position);
      Robot.elevator.setWantedPosition(position);
    }

  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.elevator.getPosition() - position) < Tuning.elevatorTolerance;
  }
}
