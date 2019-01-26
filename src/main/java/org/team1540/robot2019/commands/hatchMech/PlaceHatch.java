package org.team1540.robot2019.commands.hatchMech;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class PlaceHatch extends TimedCommand {

    public PlaceHatch() {
        super(Tuning.hatchPlaceTime);
    }

    protected void initialize() {
        Robot.hatchMech.slideOut();
        Robot.hatchMech.release();
    }

    protected void end() {
        Robot.hatchMech.slideIn();
    }

}
