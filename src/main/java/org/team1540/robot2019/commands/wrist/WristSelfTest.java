package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.rooster.util.SimpleCommand;

public class WristSelfTest extends CommandGroup {

  public static final Logger logger = Logger.getLogger(WristSelfTest.class);

  public WristSelfTest() {
    addSequential(
        new SimpleCommand("Print status", () -> logger.info("Beginning wrist self-test")));
    addSequential(new LowerWrist());
    addParallel(new WristUp());
    addSequential(new TimedCommand(1));
    addSequential(new SimpleCommand("Print status", () -> logger.info("Wrist self-test complete")));
  }

}
