package org.team1540.robot2019.commands.self_tests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.robot2019.commands.hatch.PlaceHatch;
import org.team1540.robot2019.commands.hatch.PrepGetPlaceHatch;
import org.team1540.rooster.util.SimpleCommand;

public class HatchSelfTest extends CommandGroup {

    public static final Logger logger = Logger.getLogger(HatchSelfTest.class);

    public HatchSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning hatch self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Grabbing")));
        addSequential(new PrepGetPlaceHatch());
        addSequential(new SimpleCommand("Print status", () -> logger.info("Releasing")));
        addSequential(new PlaceHatch());
        addSequential(new SimpleCommand("Print status", () -> logger.info("HatchMech self-test complete")));
    }

}
