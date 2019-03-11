package org.team1540.robot2019.commands.hatch.simple;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class RetractHatchMech extends InstantCommand {

    public RetractHatchMech() {
        requires(Robot.hatch);
    }

    protected void initialize() {
        Robot.hatch.retract();
    }

}
