package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.rooster.util.SimpleCommand;

public class HatchMechSelfTest extends CommandGroup {

    private static final Logger logger = Logger.getLogger(HatchMechSelfTest.class);

    public HatchMechSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning hatch self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Grabbing")));
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(Tuning.hatchExtendWaitTime));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(Tuning.hatchReleaseWaitTime));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Releasing")));
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
        addSequential(new SimpleCommand("Print status", () -> logger.info("HatchMech self-test complete")));
    }

}
