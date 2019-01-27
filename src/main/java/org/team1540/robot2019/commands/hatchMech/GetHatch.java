package org.team1540.robot2019.commands.hatchMech;

import edu.wpi.first.wpilibj.command.TimedCommand;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;

// NOT USING THIS CLASS AT THE MOMENT
public class GetHatch extends TimedCommand {

    public GetHatch() {
        super(Tuning.hatchGetTime);
    }

    protected void initialize() {
        Robot.hatchMech.slideOut();
        Robot.hatchMech.attatch();
    }

    protected void end() {
        Robot.hatchMech.slideIn();
    }

}
