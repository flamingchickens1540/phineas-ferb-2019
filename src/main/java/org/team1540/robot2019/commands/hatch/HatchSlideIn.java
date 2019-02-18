package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

public class HatchSlideIn extends InstantCommand {

    public HatchSlideIn() {
        requires(Robot.hatchMech);
    }

    protected void initialize() {
        Robot.hatchMech.slideIn();
    }

}
