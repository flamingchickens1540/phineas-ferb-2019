package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;

public class AutoIntakeCargo extends Command {

    private static final Logger logger = Logger.getLogger(AutoIntakeCargo.class);

    public AutoIntakeCargo() {
//    super(Tuning.intakeTimeout);
        requires(Robot.cargoIntake);
    }

    @Override
    protected void initialize() {
        logger.debug("CargoIntake starting");
        Robot.cargoIntake.startIntaking();
    }

    @Override
    protected void end() {
        if (isTimedOut()) {
            logger.debug("CargoIntake stopping due to timeout");
        } else {
            logger.debug("CargoIntake stopping due to detected ball");
        }
        Robot.cargoIntake.stop();
    }

    @Override
    protected boolean isFinished() {
        return Robot.cargoIntake.hasBall() || isTimedOut();
    }
}
