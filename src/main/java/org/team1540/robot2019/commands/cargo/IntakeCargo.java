package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class IntakeCargo extends Command {

    private static final Logger logger = Logger.getLogger(IntakeCargo.class);

    public IntakeCargo() {
//    super(Tuning.intakeTimeout);
        requires(Robot.cargoMechanism);
    }

    @Override
    protected void initialize() {
        logger.debug("CargoMech starting");
        Robot.cargoMechanism.startIntaking();
    }

    @Override
    protected void end() {
        if (isTimedOut()) {
            logger.debug("CargoMech stopping due to timeout");
        } else {
            logger.debug("CargoMech stopping due to detected ball");
        }
        Robot.cargoMechanism.stop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.cargoMechanism.hasBall() || isTimedOut();
    }
}
