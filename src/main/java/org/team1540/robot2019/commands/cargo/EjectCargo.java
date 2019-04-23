package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class EjectCargo extends TimedCommand {

    private static final Logger logger = Logger.getLogger(EjectCargo.class);

    public EjectCargo() {
        super(Tuning.cargoEjectTime);
        requires(Robot.cargoMech);
    }

    @Override
    protected void initialize() {
        logger.debug("Ejecting ball");
        Robot.cargoMech.startEjecting();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        logger.debug("Stopped ejecting");
        Robot.cargoMech.stop();
    }
}
