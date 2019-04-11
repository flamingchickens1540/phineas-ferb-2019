package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.wrist.WristDown;

public class FloorIntakeCargo extends CommandGroup {

    public FloorIntakeCargo() {
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
        addSequential(new MoveElevatorToZero());
        addParallel(new IntakeCargo());
        addSequential(new WristDown());
    }
}
