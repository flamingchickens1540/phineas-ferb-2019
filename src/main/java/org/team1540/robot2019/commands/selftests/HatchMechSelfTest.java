package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.hatch.ExtendHatch;
import org.team1540.robot2019.commands.hatch.PlaceHatch;
import org.team1540.rooster.util.SimpleCommand;

public class HatchMechSelfTest extends CommandGroup {

    public static final Logger logger = Logger.getLogger(HatchMechSelfTest.class);

    public HatchMechSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning hatch self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Grabbing")));
        addSequential(new ExtendHatch());
        addSequential(new SimpleCommand("Print status", () -> logger.info("Releasing")));
        addSequential(new PlaceHatch());
        addSequential(new SimpleCommand("Print status", () -> logger.info("HatchMech self-test complete")));
    }

}
