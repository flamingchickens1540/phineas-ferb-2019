package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.subgroups.GrabHatchThenRetract;

public class GrabHatchSequence extends CommandGroup {

    public GrabHatchSequence() {
        this(null);
    }

    public GrabHatchSequence(Runnable onSensorTrip) {
        addParallel(new MoveElevatorToPosition(Tuning.elevatorHatchIntakePosition));
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.2));
        addSequential(new GrabHatchThenRetract(0.5));
    }
}
