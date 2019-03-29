package org.team1540.robot2019.commands.selftests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class CargoMechSelfTest extends CommandGroup {

    private static final Logger logger = Logger.getLogger(CargoMechSelfTest.class);

    public CargoMechSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning intake self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running intake in")));
        addSequential(new SimpleCommand("Run In", Robot.intake::startIntaking, Robot.intake));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running intake out")));
        addSequential(new SimpleCommand("Run Out", Robot.intake::startEjecting, Robot.intake));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Stopping")));
        addSequential(new SimpleCommand("Stop intake", Robot.intake::stop, Robot.intake));
        addSequential(new SimpleCommand("Check Sensor", () -> {
            if (Robot.intake.hasBall()) {
                logger.warn("Ball sensor is still tripped after ejecting");
            }
        }, Robot.intake));
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Intake self-test complete")));
    }
}
