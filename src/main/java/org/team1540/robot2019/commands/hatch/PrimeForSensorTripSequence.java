package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class PrimeForSensorTripSequence extends CommandGroup {


    public PrimeForSensorTripSequence() {
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.5));
        addSequential(new WaitForExtendTrip());
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.2));
        addSequential(new RetractHatchMech());
    }
}
