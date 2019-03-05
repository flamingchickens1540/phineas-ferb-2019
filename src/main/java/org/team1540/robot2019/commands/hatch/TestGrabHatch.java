package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatch;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatch;

public class TestGrabHatch extends CommandGroup {

    public TestGrabHatch() {
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatch());
        addSequential(new TankDriveForTimePercent(0.2, 0.3));
        addSequential(new WaitCommand(0.2));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.2));
        addSequential(new RetractHatch());
    }

}
