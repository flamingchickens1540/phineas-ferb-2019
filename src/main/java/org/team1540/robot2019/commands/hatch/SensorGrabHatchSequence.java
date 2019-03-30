package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class SensorGrabHatchSequence extends CommandGroup {

    public SensorGrabHatchSequence() {
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForExtendSensorTrip());
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.3));
        addSequential(new RetractHatchMech());
    }
}
