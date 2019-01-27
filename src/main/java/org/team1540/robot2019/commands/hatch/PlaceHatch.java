package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

public class PlaceHatch extends TimedCommand { // TODO: update to match new mech

    public PlaceHatch() {
        super(Tuning.hatchPlaceTime);
    }

    protected void initialize() {
        Robot.hatchMech.placeHatch();
    }

}
