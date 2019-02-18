package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.rooster.util.SimpleLoopCommand;

public class WristUpOrHold extends ConditionalCommand {

    public WristUpOrHold() {
        super(new WristUp(),
            new SimpleLoopCommand("Hold Wrist", () -> Robot.wrist.set(Tuning.wristHoldThrot),
                Robot.wrist));
    }

    @Override
    protected boolean condition() {
        return Robot.wrist.isAtBtm();
    }
}
