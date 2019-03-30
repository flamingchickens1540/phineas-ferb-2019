package org.team1540.robot2019.commands.wrist;

import edu.wpi.first.wpilibj.command.Command;
import org.team1540.robot2019.Robot;

public class RecoverWrist extends Command {

    public RecoverWrist() {
        requires(Robot.wrist);
    }

    @Override
    protected void initialize() {
        Robot.wrist.set(1);

    }

    @Override
    protected void end() {
        Robot.wrist.set(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }


}
