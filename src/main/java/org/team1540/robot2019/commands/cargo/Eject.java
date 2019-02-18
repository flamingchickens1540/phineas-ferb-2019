package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class Eject extends TimedCommand {

    private static final Logger logger = Logger.getLogger(Eject.class);

    public Eject() {
        super(Tuning.intakeEjectTime);
        requires(Robot.cargoMechanism);
    }

    @Override
    protected void initialize() {
        logger.debug("Ejecting ball");
        Robot.cargoMechanism.startEjecting();
    }

    @Override
    protected void end() {
        logger.debug("Stopped ejecting");

        Robot.cargoMechanism.stop();
    }
}