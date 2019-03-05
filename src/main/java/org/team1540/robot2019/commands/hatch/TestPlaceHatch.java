package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.commands.drivetrain.TankDriveForTimePercent;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatch;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatch;

public class TestPlaceHatch extends CommandGroup {

    public TestPlaceHatch() {
        addSequential(new GrabHatch());
        addSequential(new ExtendHatch());
        addSequential(new TankDriveForTimePercent(0.3, 0.3));
        addSequential(new WaitCommand(0.2));
        addSequential(new ReleaseHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new RetractHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
    }

}
