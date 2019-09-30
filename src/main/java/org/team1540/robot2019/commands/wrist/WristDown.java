package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class WristDown extends Command {

    private static final Logger logger = Logger.getLogger(WristDown.class);

    public WristDown() {
        super(Tuning.wristLowerTimeout);

        requires(Robot.wrist);
    }

    @Override
    protected void initialize() {
        if (!Robot.wrist.isAtBtm()) {
            logger.debug("Moving wrist down...");
            Robot.wrist.clearBtmFlag();
            Robot.wrist.clearMidFlag();
            Robot.wrist.set(Tuning.wristDownTravelPwrThrot);
        } else {
            logger.debug("Wrist already at bottom!");
        }
    }

    @Override
    protected void execute() {
        if (Robot.wrist.getMidFlag()) {
            logger.debug("Braking...");
            Robot.wrist.set(Tuning.wristDownTravelBrakeThrot);
        }
    }

    @Override
    protected void end() {
        if (isTimedOut()) {
            logger.debug("Finished because of timeout!");
        } else {
            logger.debug("Finished because wrist is at bottom!");
        }
        Robot.wrist.set(Tuning.wristHoldThrot);
    }

    @Override
    protected boolean isFinished() {
        return Robot.wrist.getBtmFlag() || Robot.wrist.isAtBtm() || isTimedOut();
    }
}
