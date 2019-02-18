package org.team1540.robot2019.commands.self_tests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.apache.log4j.Logger;
import org.team1540.rooster.util.SimpleCommand;

public class SelfTest extends CommandGroup {

    public static final Logger logger = Logger.getLogger(SelfTest.class);

    public SelfTest() {
        addSequential(new SimpleCommand("Print status", () -> logger.info("Starting robot self-test")));
        addSequential(new HatchSelfTest());
        addSequential(new CargoIntakeSelfTest());
        addSequential(new WristSelfTest());
        addSequential(new ElevatorSelfTest());
        addSequential(new SimpleCommand("Print status", () -> logger.info("Robot self-test complete")));
    }
}
