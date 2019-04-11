package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import org.apache.log4j.Logger;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.util.SimpleLoopCommand;

public class WristUpOrHold extends ConditionalCommand {

    private static final Logger logger = Logger.getLogger(WristUpOrHold.class);

    public WristUpOrHold() {
        super(new SimpleWristUp(),
            new SimpleLoopCommand("Hold Wrist", () -> Robot.wrist.set(-Tuning.wristHoldThrot),
                Robot.wrist));
    }

    @Override
    protected boolean condition() {
        boolean atBtm = Robot.wrist.isAtBtm();
        if (atBtm) {
            logger.debug("Wrist at bottom! Moving up...");
        } else {
            logger.debug("Wrist already not at bottom! Holding...");
        }
        return atBtm;
    }
}
