package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Robot;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;
import org.team1540.robot2019.commands.wrist.WristDown;
import org.team1540.rooster.util.SimpleCommand;

public class FloorIntakeCargo extends CommandGroup {

    public FloorIntakeCargo() {
        addParallel(new SimpleCommand("Retract Hatch", Robot.hatch::retract, Robot.hatch));
        addSequential(new MoveElevatorToZero());
        addParallel(new IntakeCargo());
        addSequential(new WristDown());
    }
}
