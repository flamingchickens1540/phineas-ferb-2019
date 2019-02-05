package org.team1540.robot2019.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class MoveArmsToStart extends Command {

  public MoveArmsToStart() {
    requires(Robot.climber);
  }

  @Override
  protected void initialize() {
    Robot.climber.setArmPosition(Tuning.climberGyroStartPos);
  }

  @Override
  protected void end() {
    Robot.climber.setArms(0);
  }

  @Override
  protected boolean isFinished() {
    return Math.abs(Robot.climber.getPosition() - Tuning.climberGyroStartPos)
        < Tuning.climberTolerance;
  }
}
