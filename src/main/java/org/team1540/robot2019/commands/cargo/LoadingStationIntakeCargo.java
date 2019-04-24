package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;
import org.team1540.robot2019.commands.wrist.WristUp;

public class LoadingStationIntakeCargo extends CommandGroup {

    public LoadingStationIntakeCargo() {
        addSequential(new ReleaseHatch());
        addSequential(new RetractHatchMech());
        addSequential(new WristUp());
//        addParallel(new MoveElevatorToPosition(Tuning.elevatorLoadingStationPosition));
        addSequential(new IntakeCargo());
        addSequential(new MoveElevatorToZero());
    }
}
