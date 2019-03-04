package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;

public class TestPlaceHatch extends CommandGroup {

    public TestPlaceHatch() {
        addSequential(new GrabHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new TankDriveForTimePercent(0.2, 0.3));
        addSequential(new WaitCommand(0.2));
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new RetractHatchMech());
    }

}
