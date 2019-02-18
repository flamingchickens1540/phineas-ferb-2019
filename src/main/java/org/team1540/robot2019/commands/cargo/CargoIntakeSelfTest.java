package org.team1540.robot2019.commands.cargo;

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
            new SimpleCommand("Print status", () -> logger.info("Beginning cargoIntake self-test")));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoIntake in")));
        addSequential(new SimpleCommand("Run In", Robot.cargoIntake::startIntaking, Robot.cargoIntake));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Running cargoIntake out")));
        addSequential(new SimpleCommand("Run Out", Robot.cargoIntake::startEjecting, Robot.cargoIntake));
        addSequential(new TimedCommand(1));
        addSequential(new SimpleCommand("Print status", () -> logger.info("Stopping")));
        addSequential(new SimpleCommand("Stop cargoIntake", Robot.cargoIntake::stop, Robot.cargoIntake));
        addSequential(new SimpleCommand("Check Sensor", () -> {
            if (Robot.cargoIntake.hasBall()) {
                DriverStation.reportWarning("Ball sensor is still tripped after ejecting", false);
            }
        }, Robot.cargoIntake));
        addSequential(
            new SimpleCommand("Print status", () -> logger.info("CargoIntake self-test complete")));
    }
}
