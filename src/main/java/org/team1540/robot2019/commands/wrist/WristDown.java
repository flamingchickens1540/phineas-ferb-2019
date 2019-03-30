package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class WristDown extends Command {

    public WristDown() {
        super(Tuning.wristLowerTimeout);

        requires(Robot.wrist);
    }

    @Override
    protected void initialize() {
        if (!Robot.wrist.isAtBtm()) {
            Robot.wrist.clearBtmFlag();
            Robot.wrist.clearMidFlag();
            Robot.wrist.set(Tuning.wristDownTravelPwrThrot);
        }
    }

    @Override
    protected void execute() {
        if (Robot.wrist.getMidFlag()) {
            Robot.wrist.set(Tuning.wristDownTravelBrakeThrot);
        }
    }

    @Override
    protected void end() {
        Robot.wrist.set(0);
    }

    @Override
    protected boolean isFinished() {
        return Robot.wrist.getBtmFlag() || Robot.wrist.isAtBtm() || isTimedOut();
    }
}
