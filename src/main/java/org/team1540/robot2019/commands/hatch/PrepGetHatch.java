package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.team1540.robot2019.Robot;

/**
 * Quickly retracts hatch mech. This won't block a command group to wait for the mechanism like PlaceHatch will; use before floor-intaking something.
 */
public class PrepGetHatch extends InstantCommand {

    public PrepGetHatch() {
        requires(Robot.hatch);
    }

    @Override
    protected void execute() {
        Robot.hatch.extend();
        Robot.hatch.release();
    }
}
