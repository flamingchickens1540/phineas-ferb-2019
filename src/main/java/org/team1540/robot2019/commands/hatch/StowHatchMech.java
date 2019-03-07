package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

/**
 * Quickly retracts hatch mech
 */
public class StowHatchMech extends InstantCommand {

    public StowHatchMech() {
        requires(Robot.hatch);
    }

    @Override
    protected void execute() {
        Robot.hatch.release();
        Robot.hatch.retract();
    }
}
