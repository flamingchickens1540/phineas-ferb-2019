package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class Eject extends Command {

    private static final Logger logger = Logger.getLogger(Eject.class);

    public Eject() {
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        logger.debug("Ejecting ball");
        Robot.intake.startEjecting();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        logger.debug("Stopped ejecting");
        Robot.intake.stop();
    }
}
