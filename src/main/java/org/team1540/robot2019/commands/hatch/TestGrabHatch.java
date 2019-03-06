package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class TestGrabHatch extends CommandGroup {

    public TestGrabHatch() {
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
//        addSequential(new SimpleCommand("Drive", () -> new TankDriveForTimePercent(0.2, 0.3).start()));
//        addSequential(new TimedCommand(0.2));
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
