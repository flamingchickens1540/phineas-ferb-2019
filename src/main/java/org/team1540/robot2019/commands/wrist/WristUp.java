package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class WristUp extends Command {

    @Override
    protected void initialize() {
        Robot.wrist.clearMidFlag();
        Robot.wrist.set(Tuning.wristUpTravelThrot);
    }

    @Override
    protected void end() {
        Robot.wrist.set(Tuning.wristHoldThrot);
    }

    @Override
    protected boolean isFinished() {
        return Robot.wrist.getMidFlag();
    }
}
