package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

class SimpleWristUp extends Command {

    private static final Logger logger = Logger.getLogger(SimpleWristUp.class);

    @Override
    protected void initialize() {
        logger.debug("Moving wrist up...");
        Robot.wrist.clearMidFlag();
        Robot.wrist.set(-Tuning.wristUpTravelThrot);
    }

    @Override
    protected void end() {
        logger.debug("Mid flag reached! Holding...");
        Robot.wrist.set(-Tuning.wristHoldThrot);
    }

    @Override
    protected boolean isFinished() {
        return Robot.wrist.getMidFlag();
    }
}
