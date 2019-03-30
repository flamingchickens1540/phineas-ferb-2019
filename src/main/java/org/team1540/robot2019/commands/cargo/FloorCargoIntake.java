package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.StowHatchMech;
import org.team1540.robot2019.commands.wrist.WristDown;

public class FloorCargoIntake extends CommandGroup {

    public FloorCargoIntake() {
        addParallel(new StowHatchMech());
        addSequential(new MoveElevatorToZero());
        addParallel(new IntakeCargo());
        addSequential(new WristDown());
    }
}
