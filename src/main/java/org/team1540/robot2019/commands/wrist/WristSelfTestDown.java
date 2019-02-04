package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.subsystems.Wrist.WristState;

public class WristSelfTestDown extends TimedCommand {

  public static final Logger logger = Logger.getLogger(WristSelfTestDown.class);

  public WristSelfTestDown() {
    super(Tuning.wristTestDownTime);
    requires(Robot.wrist);
  }

  @Override
  protected void initialize() {
    logger.info("Performing wrist down movement self-test");
    Robot.wrist.moveDown();
  }

  @Override
  protected void end() {
    if (Robot.wrist.getState() != WristState.OFF_DOWN) {
      DriverStation.reportWarning("Wrist did not move down correctly", false);

      Robot.wrist.handleDisable(); // effectively stops the wrist
    } else {
      logger.info("Wrist down movement nominal");
    }
  }
}
