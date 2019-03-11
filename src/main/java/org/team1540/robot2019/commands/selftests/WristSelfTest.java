package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.robot2019.commands.wrist.WristUpOrHold;
import org.team1540.rooster.util.SimpleCommand;

public class WristSelfTest extends CommandGroup {

    public static final Logger logger = Logger.getLogger(WristSelfTest.class);

    public WristSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning wrist self-test")));
        addSequential(new WristDown());
        addSequential(new WaitCommand(1));
        addParallel(new WristUpOrHold());
        addSequential(new WaitCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Wrist self-test complete")));
    }

}
