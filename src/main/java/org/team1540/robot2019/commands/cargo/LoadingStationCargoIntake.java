package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.rooster.util.SimpleCommand;

public class LoadingStationCargoIntake extends CommandGroup {

    public LoadingStationCargoIntake() {
        addParallel(new SimpleCommand("Retract Hatch", Robot.hatch::retract, Robot.hatch));
        addSequential(new MoveElevatorToPosition(Tuning.elevatorLoadingStationPosition));
        addSequential(new IntakeCargo());
        addSequential(new MoveElevatorToZero());
    }
}
