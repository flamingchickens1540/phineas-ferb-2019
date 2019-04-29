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
            new SimpleCommand("Print status", () -> logger.info("Beginning cargoMech self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoMech in")));
        addSequential(new SimpleCommand("Run In", Robot.cargoMech::startIntaking, Robot.cargoMech));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoMech out")));
        addSequential(new SimpleCommand("Run Out", Robot.cargoMech::startEjecting, Robot.cargoMech));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Stopping")));
        addSequential(new SimpleCommand("Stop cargo mech", Robot.cargoMech::stop, Robot.cargoMech));
        addSequential(new SimpleCommand("Check Sensor", () -> {
            if (Robot.cargoMech.hasBall()) {
                logger.warn("Ball sensor is still tripped after ejecting");
            }
        }, Robot.cargoMech));
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("CargoMech self-test complete")));
    }
}
