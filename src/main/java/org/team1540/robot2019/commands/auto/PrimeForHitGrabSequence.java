package org.team1540.robot2019.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class PrimeForHitGrabSequence extends CommandGroup {


    public PrimeForHitGrabSequence() {
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
        addSequential(new WaitForHit());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.2));
        addSequential(new GrabHatch());
    }
}
