package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class Intake extends Command {

    private static final Logger logger = Logger.getLogger(Intake.class);

    public Intake() {
        requires(Robot.cargoMechanism);
    }

    @Override
    protected void initialize() {
        logger.debug("Intake starting");
        Robot.cargoMechanism.startIntaking();
    }

    @Override
    protected void end() {
        if (isTimedOut()) {
            logger.debug("Intake stopping due to timeout");
        } else {
            logger.debug("Intake stopping due to detected ball");
        }
        Robot.cargoMechanism.stop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.cargoMechanism.hasBall() || isTimedOut();
    }
}
