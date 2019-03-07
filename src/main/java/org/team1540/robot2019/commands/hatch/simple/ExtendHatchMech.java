package org.team1540.robot2019.commands.hatch.simple;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class ExtendHatchMech extends InstantCommand {

    public ExtendHatchMech() {
        requires(Robot.hatch);
    }

    protected void initialize() {
        Robot.hatch.extend();
    }

}
