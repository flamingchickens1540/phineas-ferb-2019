package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class TestGrabHatch extends CommandGroup {

    public TestGrabHatch() {
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.2));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.2));
        addSequential(new RetractHatchMech());
    }

    @Override
    protected void interrupted() {
        new GrabHatch().start();
        new RetractHatchMech().start();
    }
}
