package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class RetractHatch extends InstantCommand {

    public RetractHatch() {
        requires(Robot.hatch);
    }

    protected void initialize() {
        Robot.hatch.retract();
    }

}
