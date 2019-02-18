package org.team1540.robot2019.commands.self_tests;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.rooster.util.SimpleCommand;

public class CargoIntakeSelfTest extends CommandGroup {

    public static final Logger logger = Logger.getLogger(CargoIntakeSelfTest.class);

    public CargoIntakeSelfTest() {
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Beginning cargo self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargo in")));
        addSequential(new SimpleCommand("Run In", Robot.cargo::startIntaking, Robot.cargo));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargo out")));
        addSequential(new SimpleCommand("Run Out", Robot.cargo::startEjecting, Robot.cargo));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Stopping")));
        addSequential(new SimpleCommand("Stop cargo", Robot.cargo::stop, Robot.cargo));
        addSequential(new SimpleCommand("Check Sensor", () -> {
            if (Robot.cargo.hasBall()) {
                DriverStation.reportWarning("Ball sensor is still tripped after ejecting", false);
            }
        }, Robot.cargo));
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("Cargo self-test complete")));
    }
}
