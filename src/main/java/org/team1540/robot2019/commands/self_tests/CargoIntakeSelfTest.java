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
            new SimpleCommand("Print status", () -> logger.info("Beginning cargoMechanism self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoMechanism in")));
        addSequential(new SimpleCommand("Run In", Robot.cargoMechanism::startIntaking, Robot.cargoMechanism));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoMechanism out")));
        addSequential(new SimpleCommand("Run Out", Robot.cargoMechanism::startEjecting, Robot.cargoMechanism));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Stopping")));
        addSequential(new SimpleCommand("Stop cargoMechanism", Robot.cargoMechanism::stop, Robot.cargoMechanism));
        addSequential(new SimpleCommand("Check Sensor", () -> {
            if (Robot.cargoMechanism.hasBall()) {
                DriverStation.reportWarning("Ball sensor is still tripped after ejecting", false);
            }
        }, Robot.cargoMechanism));
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("CargoMech self-test complete")));
    }
}
