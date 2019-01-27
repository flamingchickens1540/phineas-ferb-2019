package org.team1540.robot2019.commands.gamepieceGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team1540.robot2019.Tuning;
import org.team1540.robot2019.commands.intake.Eject;

public class EjectThenDown extends CommandGroup {

    public EjectThenDown() {
        addSequential(new Eject());
        addSequential(new MoveElevatorToPosition(Tuning.elevatorDownPosition));
    }

}
