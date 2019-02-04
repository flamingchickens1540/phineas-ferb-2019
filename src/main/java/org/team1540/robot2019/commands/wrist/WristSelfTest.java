package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.rooster.util.SimpleCommand;

public class WristSelfTest extends CommandGroup {

  public static final Logger logger = Logger.getLogger(WristSelfTest.class);

  public WristSelfTest() {
    addSequential(
        new SimpleCommand("Print status", () -> logger.info("Beginning wrist self-test")));
    addSequential(new WristSelfTestDown());
    addSequential(new WristSelfTestUp());
    addSequential(new SimpleCommand("Print status", () -> logger.info("Wrist self-test complete")));
  }

}
