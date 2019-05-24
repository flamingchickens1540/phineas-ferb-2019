package org.team1540.robot2019.commands.hatch.floor;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.subgroups.StowHatchMech;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleLoopCommand;

public class PrepHatchFloorGrab extends CommandGroup {

    public PrepHatchFloorGrab() {
        addSequential(new StowHatchMech());
        addSequential(new MoveElevatorToZero());
        addSequential(new WristDown());
        addSequential(new WaitCommand(Tuning.hatchPrepFloorWaitTime));
        addSequential(new ExtendHatchMech());
        addSequential(new SimpleLoopCommand("Wait until interrupted", () -> {
        }));
    }
}
