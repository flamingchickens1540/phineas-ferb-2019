package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.apache.log4j.Logger;
import org.team1540.rooster.util.SimpleCommand;

public class SelfTest extends CommandGroup {

    private static final Logger logger = Logger.getLogger(SelfTest.class);

    public SelfTest() {
        addSequential(new SimpleCommand("Print status", () -> logger.info("Starting robot self-test")));
        addSequential(new HatchMechSelfTest());
        addSequential(new CargoMechSelfTest());
        addSequential(new WristSelfTest());
//        addSequential(new ElevatorSelfTest());
        addSequential(new ElevatorSelfTestEachMotor(true));
        addSequential(new WaitCommand(0.5));
        addSequential(new ElevatorSelfTestEachMotor(false)); // TODO: This doesn't work-- b/c follower???
        addSequential(new DriveSelfTest(true));
        addSequential(new DriveSelfTest(false));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Robot self-test complete")));
    }
}
