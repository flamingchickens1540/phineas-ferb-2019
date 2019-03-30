package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class IntakeCargo extends Command {

    private static final Logger logger = Logger.getLogger(IntakeCargo.class);

    public IntakeCargo() {
        requires(Robot.intake);
    }

    @Override
    protected void initialize() {
        logger.debug("Intake starting");
        Robot.intake.startIntaking();
    }

    @Override
    protected void end() {
        logger.debug("Intake stopping");
        Robot.intake.stop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.intake.hasBall() || isTimedOut();
    }
}
