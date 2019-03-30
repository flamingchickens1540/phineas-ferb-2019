package org.team1540.robot2019.commands.hatch;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.elevator.MoveElevatorToPosition;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.hatch.simple.ExtendHatchMech;
import org.team1540.robot2019.commands.hatch.simple.GrabHatch;
import org.team1540.robot2019.commands.hatch.simple.ReleaseHatch;
import org.team1540.robot2019.commands.hatch.simple.RetractHatchMech;

public class GrabHatchSequence extends CommandGroup {


    public GrabHatchSequence() {
        addParallel(new MoveElevatorToPosition(Tuning.elevatorHatchIntakePosition));
        addSequential(new ReleaseHatch());
        addSequential(new ExtendHatchMech());
        addSequential(new WaitCommand(0.2));
        addSequential(new GrabHatch());
        addSequential(new WaitCommand(0.4));
        addSequential(new RetractHatchMech());
        addSequential(new WaitCommand(0.1));
        addSequential(new MoveElevatorToZero());
    }
}
