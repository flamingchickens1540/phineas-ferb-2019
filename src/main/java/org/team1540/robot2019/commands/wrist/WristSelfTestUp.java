package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Wrist.WristState;

public class WristSelfTestUp extends TimedCommand {

  public static final Logger logger = Logger.getLogger(WristSelfTestUp.class);

  public WristSelfTestUp() {
    super(Tuning.wristTestUpTime);
    requires(Robot.wrist);
  }

  @Override
  protected void initialize() {
    logger.info("Performing wrist up movement self-test");
    Robot.wrist.moveUp();
  }

  @Override
  protected void end() {
    if (Robot.wrist.getState() != WristState.OFF_UP) {
      DriverStation.reportWarning("Wrist did not move up correctly", false);

      Robot.wrist.handleDisable(); // effectively stops the wrist
    } else {
      logger.info("Wrist up movement nominal");
    }
  }
}
