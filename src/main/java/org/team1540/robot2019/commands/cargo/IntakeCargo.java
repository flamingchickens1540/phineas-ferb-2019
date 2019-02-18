package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class IntakeCargo extends Command {

    private static final Logger logger = Logger.getLogger(IntakeCargo.class);

    public IntakeCargo() {
//    super(Tuning.intakeTimeout);
        requires(Robot.cargo);
    }

    @Override
    protected void initialize() {
        logger.debug("Cargo starting");
        Robot.cargo.startIntaking();
    }

    @Override
    protected void end() {
        if (isTimedOut()) {
            logger.debug("Cargo stopping due to timeout");
        } else {
            logger.debug("Cargo stopping due to detected ball");
        }
        Robot.cargo.stop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.cargo.hasBall() || isTimedOut();
    }
}
