package org.team1540.robot2019.commands.cargo;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.commands.elevator.MoveElevatorToZero;

public class EjectCargoThenDown extends CommandGroup {

    public EjectCargoThenDown() {
        addSequential(new EjectCargo());
        addSequential(new MoveElevatorToZero());
    }

}
