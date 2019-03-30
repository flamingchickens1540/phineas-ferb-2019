package org.team1540.robot2019.commands.hatch.simple;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class ReleaseHatch extends InstantCommand {

    public ReleaseHatch() {
        requires(Robot.hatch);
    }

    @Override
    protected void execute() {
        Robot.hatch.release();
    }
}
