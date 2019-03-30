package org.team1540.robot2019.commands.hatch.simple;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class GrabHatch extends InstantCommand {

    public GrabHatch() {
        requires(Robot.hatch);
    }

    @Override
    protected void initialize() {
        Robot.hatch.grab();
    }
}
